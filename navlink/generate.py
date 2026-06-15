#!/usr/bin/env python3
"""NavLink v2 code generator.

Reads `dialect.json` (the single source of truth, spec §7) and emits, deterministically:
  - C  : navlink_msgs.h / navlink_msgs.c   (firmware + GCS)
  - Py : navlink_msgs.py                    (tools / autotuner)

Each tree gets: enums, a packed wire struct + naturally-aligned struct per message,
pack/unpack (one memcpy, spec §8.2), to_aligned/from_aligned converters, the
CRC_EXTRA table (spec §4.3), and a msgid -> {name,size,crc_extra} dispatch table.

Pure stdlib. Usage:
    python3 generate.py [--dialect dialect.json] [--out generated] [--lang c|py|both]
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys

# ── type model ──────────────────────────────────────────────────────────────
# name -> (wire_bytes, c_wire_type, c_aligned_type, py_struct_char)
# u24 is special (no C/struct primitive): 3 wire bytes, u32 aligned.
TYPES = {
    "u8":  (1, "uint8_t",  "uint8_t",  "B"),
    "i8":  (1, "int8_t",   "int8_t",   "b"),
    "u16": (2, "uint16_t", "uint16_t", "H"),
    "i16": (2, "int16_t",  "int16_t",  "h"),
    "u24": (3, None,       "uint32_t", None),
    "u32": (4, "uint32_t", "uint32_t", "I"),
    "i32": (4, "int32_t",  "int32_t",  "i"),
    "u64": (8, "uint64_t", "uint64_t", "Q"),
    "i64": (8, "int64_t",  "int64_t",  "q"),
    "f32": (4, "float",    "float",    "f"),
    "f64": (8, "double",   "double",   "d"),
    "char":(1, "char",     "char",     "s"),
}


def field_len(f):
    """Array length, or None for a scalar."""
    return f.get("len")


def field_bytes(f):
    n = field_len(f) or 1
    return TYPES[f["type"]][0] * n


def ordered_fields(msg):
    """Non-extension fields (ascending index) then extension fields (ascending index).

    This is wire order (spec §5.2, §5.5)."""
    fs = sorted(msg["fields"], key=lambda f: f["index"])
    return [f for f in fs if not f.get("extension")] + [f for f in fs if f.get("extension")]


def crc_fields(msg):
    """Fields that participate in CRC_EXTRA: non-extension, ascending index (spec §4.3)."""
    return sorted((f for f in msg["fields"] if not f.get("extension")),
                  key=lambda f: f["index"])


def wire_size(msg):
    return sum(field_bytes(f) for f in ordered_fields(msg))


# Command range (spec §9): msgid 0x002000-0x002FFF. Per spec §12.1 the receiver
# MUST answer a command with COMMAND_ACK, so messages in this range require an
# ack by default. Any message may override that with an explicit "ack" boolean.
def requires_ack(msg):
    default = 0x2000 <= msg["msgid"] <= 0x2FFF
    return bool(msg.get("ack", default))


def canonical_values(msg):
    """Deterministic per-field test values, in wire order.

    Shared by the C parity emitter (baked in as literals) and the Python tests,
    so both sides exercise identical non-trivial values: signed negatives, arrays,
    u24, char strings, floats. Values stay small (and exactly representable) so the
    encoding is unambiguous across languages."""
    out = []
    for o, f in enumerate(ordered_fields(msg)):
        t, ln = f["type"], field_len(f)
        if t == "char":
            out.append("".join(chr(65 + ((o + j) % 26)) for j in range(ln)))
        elif t in ("f32", "f64"):
            sgn = -1.0 if (o % 2) else 1.0
            out.append(sgn * (o + 1) if ln is None
                       else [sgn * ((o + 1) + j * 0.25) for j in range(ln)])
        else:
            signed = t in ("i8", "i16", "i32", "i64")

            def iv(j, _o=o, _s=signed):
                m = (_o * 7 + j * 3 + 1) % 100
                return -m if (_s and _o % 2) else m

            out.append(iv(0) if ln is None else [iv(j) for j in range(ln)])
    return out


def c_literal(ftype, v):
    if ftype == "f32":
        return repr(float(v)) + "f"
    if ftype == "f64":
        return repr(float(v))
    return str(int(v))


# ── CRC ──────────────────────────────────────────────────────────────────────
def crc_accumulate(b, crc):
    t = b ^ (crc & 0xFF)
    t = (t ^ (t << 4)) & 0xFF
    return ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF


def crc16(data, crc=0xFFFF):
    for b in data:
        crc = crc_accumulate(b, crc)
    return crc


def crc_extra(msg):
    """1-byte per-message layout signature (spec §4.3)."""
    crc = 0xFFFF
    for ch in msg["name"].encode("ascii"):
        crc = crc_accumulate(ch, crc)
    crc = crc_accumulate(0x20, crc)
    for f in crc_fields(msg):
        for ch in f["type"].encode("ascii"):   # wire_type_name == declared type (spec §4.3, §5.4)
            crc = crc_accumulate(ch, crc)
        crc = crc_accumulate(0x20, crc)
        for ch in f["name"].encode("ascii"):
            crc = crc_accumulate(ch, crc)
        crc = crc_accumulate(0x20, crc)
        ln = field_len(f)
        if ln is not None:
            crc = crc_accumulate(ln & 0xFF, crc)
    return ((crc & 0xFF) ^ (crc >> 8)) & 0xFF


# ── validation (light; the JSON Schema is the full check) ────────────────────
def validate(d):
    errs = []
    seen_ids, seen_names = {}, {}
    for m in d["messages"]:
        mid, name = m["msgid"], m["name"]
        if mid in seen_ids:
            errs.append(f"duplicate msgid {mid}: {name} vs {seen_ids[mid]}")
        seen_ids[mid] = name
        if name in seen_names:
            errs.append(f"duplicate message name {name}")
        seen_names[name] = True
        if not (0 <= mid <= 0x7FFFFF):
            errs.append(f"{name}: msgid {mid} outside core half 0x000000-0x7FFFFF (spec §9)")
        # contiguous, unique indices across non-extension fields
        nonext = [f["index"] for f in m["fields"] if not f.get("extension")]
        if sorted(nonext) != list(range(len(nonext))):
            errs.append(f"{name}: non-extension indices not contiguous from 0: {sorted(nonext)}")
        fnames = [f["name"] for f in m["fields"]]
        if len(fnames) != len(set(fnames)):
            errs.append(f"{name}: duplicate field name")
        # ack-requiring messages carry the command header: the dispatch reads
        # req_seq to correlate the COMMAND_ACK it auto-emits (spec §12.1).
        if requires_ack(m):
            rq = next((f for f in m["fields"] if f["name"] == "req_seq"), None)
            if rq is None or rq["type"] != "u8":
                errs.append(f"{name}: requires ack but has no req_seq:u8 field (spec §12.1)")
        for f in m["fields"]:
            if f["type"] not in TYPES:
                errs.append(f"{name}.{f['name']}: unknown type {f['type']}")
            if f.get("enum") and f["enum"] not in d.get("enums", {}):
                errs.append(f"{name}.{f['name']}: unknown enum {f['enum']}")
        if all(f["type"] in TYPES for f in m["fields"]):   # size needs known types
            size = wire_size(m)
            if size > 255:
                errs.append(f"{name}: wire size {size} > 255 (single-frame limit, spec §3.6)")
    return errs


# ── naming helpers ────────────────────────────────────────────────────────────
def c_msg_prefix(name):
    return "navlink_" + name.lower()


def pascal(name):
    return "".join(p.capitalize() for p in name.split("_"))


# ── C generation ──────────────────────────────────────────────────────────────
def c_member(f, aligned):
    t = f["type"]
    ln = field_len(f)
    if t == "u24":
        ctype = "uint32_t" if aligned else "uint8_t"
        if aligned:
            return f"    uint32_t {f['name']};"
        return f"    uint8_t  {f['name']}[3];"
    ctype = TYPES[t][2] if aligned else TYPES[t][1]
    if ln is not None:
        return f"    {ctype} {f['name']}[{ln}];"
    return f"    {ctype} {f['name']};"


def gen_c_header(d):
    L = []
    p = L.append
    p("/* GENERATED by navlink/generate.py — DO NOT EDIT. Source: dialect.json */")
    p("#ifndef NAVLINK_MSGS_H")
    p("#define NAVLINK_MSGS_H")
    p("#include <stdint.h>")
    p("#include <stddef.h>")
    p("#include <string.h>")
    p("")
    p('#ifdef __cplusplus')
    p('extern "C" {')
    p('#endif')
    p("")
    # enums — typedef suffix `_e` keeps them out of the message struct namespace
    # (a `flight_mode` enum and a `FLIGHT_MODE` message would otherwise collide).
    for ename, e in d.get("enums", {}).items():
        p(f"/* {e.get('doc','')} */".strip())
        p(f"typedef enum {{")
        for ent in e["entries"]:
            p(f"    NAVLINK_{ename.upper()}_{ent['name']} = {ent['value']},")
        p(f"}} navlink_{ename}_e;")
        p("")
    # per-message
    for m in d["messages"]:
        pre = c_msg_prefix(m["name"])
        p(f"/* ===== {m['name']} (msgid {m['msgid']}) ===== */")
        if m.get("doc"):
            p(f"/* {m['doc']} */")
        p(f"#define NAVLINK_MSGID_{m['name']} {m['msgid']}u")
        p(f"#define NAVLINK_CRC_EXTRA_{m['name']} {crc_extra(m)}u")
        p(f"#define NAVLINK_WIRE_SIZE_{m['name']} {wire_size(m)}u")
        p(f"#define NAVLINK_ACK_{m['name']} {1 if requires_ack(m) else 0}u  /* receiver owes COMMAND_ACK */")
        # wire struct (packed)
        p("typedef struct __attribute__((packed)) {")
        for f in ordered_fields(m):
            p(c_member(f, aligned=False))
        p(f"}} {pre}_wire_t;")
        # aligned struct
        p("typedef struct {")
        for f in ordered_fields(m):
            p(c_member(f, aligned=True))
        p(f"}} {pre}_t;")
        # prototypes
        p(f"size_t {pre}_pack(uint8_t *buf, const {pre}_wire_t *w);")
        p(f"void   {pre}_unpack({pre}_wire_t *w, const uint8_t *buf, size_t len);")
        p(f"void   {pre}_to_aligned({pre}_t *a, const {pre}_wire_t *w);")
        p(f"void   {pre}_from_aligned({pre}_wire_t *w, const {pre}_t *a);")
        p("")
    # dispatch table
    p("typedef struct {")
    p("    uint32_t msgid;")
    p("    uint16_t wire_size;")
    p("    uint8_t  crc_extra;")
    p("    uint8_t  requires_ack; /* receiver owes a COMMAND_ACK (spec §12.1) */")
    p("    const char *name;")
    p("} navlink_msg_info_t;")
    p(f"#define NAVLINK_MSG_COUNT {len(d['messages'])}u")
    p("extern const navlink_msg_info_t navlink_msg_table[NAVLINK_MSG_COUNT]; /* sorted by msgid */")
    p("const navlink_msg_info_t *navlink_msg_info(uint32_t msgid);")
    p("")
    p("/* CRC-16/MCRF4XX accumulator (spec §4.1). */")
    p("static inline void navlink_crc_accumulate(uint8_t b, uint16_t *crc) {")
    p("    uint8_t t = b ^ (uint8_t)(*crc & 0xFF);")
    p("    t ^= (uint8_t)(t << 4);")
    p("    *crc = (*crc >> 8) ^ ((uint16_t)t << 8) ^ ((uint16_t)t << 3) ^ ((uint16_t)t >> 4);")
    p("}")
    p("")
    p(gen_c_frame_decls(d))
    p("")
    p('#ifdef __cplusplus')
    p('}')
    p('#endif')
    p("#endif /* NAVLINK_MSGS_H */")
    return "\n".join(L) + "\n"


def gen_c_source(d):
    L = []
    p = L.append
    p("/* GENERATED by navlink/generate.py — DO NOT EDIT. Source: dialect.json */")
    p('#include "navlink_msgs.h"')
    p("")
    for m in d["messages"]:
        pre = c_msg_prefix(m["name"])
        p(f"size_t {pre}_pack(uint8_t *buf, const {pre}_wire_t *w) {{")
        p(f"    memcpy(buf, w, sizeof *w);")
        p(f"    return sizeof *w;")
        p("}")
        p(f"void {pre}_unpack({pre}_wire_t *w, const uint8_t *buf, size_t len) {{")
        p(f"    memset(w, 0, sizeof *w);")
        p(f"    memcpy(w, buf, len <= sizeof *w ? len : sizeof *w);")
        p("}")
        p(f"void {pre}_to_aligned({pre}_t *a, const {pre}_wire_t *w) {{")
        for f in ordered_fields(m):
            nm, t, ln = f["name"], f["type"], field_len(f)
            if t == "u24":
                p(f"    a->{nm} = (uint32_t)w->{nm}[0] | ((uint32_t)w->{nm}[1] << 8) | ((uint32_t)w->{nm}[2] << 16);")
            elif ln is not None:
                p(f"    memcpy(a->{nm}, w->{nm}, sizeof a->{nm});")
            else:
                p(f"    a->{nm} = w->{nm};")
        p("}")
        p(f"void {pre}_from_aligned({pre}_wire_t *w, const {pre}_t *a) {{")
        for f in ordered_fields(m):
            nm, t, ln = f["name"], f["type"], field_len(f)
            if t == "u24":
                p(f"    w->{nm}[0] = (uint8_t)(a->{nm} & 0xFF);")
                p(f"    w->{nm}[1] = (uint8_t)((a->{nm} >> 8) & 0xFF);")
                p(f"    w->{nm}[2] = (uint8_t)((a->{nm} >> 16) & 0xFF);")
            elif ln is not None:
                p(f"    memcpy(w->{nm}, a->{nm}, sizeof w->{nm});")
            else:
                p(f"    w->{nm} = a->{nm};")
        p("}")
        p("")
    # dispatch table, sorted by msgid
    p("const navlink_msg_info_t navlink_msg_table[NAVLINK_MSG_COUNT] = {")
    for m in sorted(d["messages"], key=lambda m: m["msgid"]):
        p(f'    {{ {m["msgid"]}u, {wire_size(m)}u, {crc_extra(m)}u, {1 if requires_ack(m) else 0}u, "{m["name"]}" }},')
    p("};")
    p("")
    p("const navlink_msg_info_t *navlink_msg_info(uint32_t msgid) {")
    p("    size_t lo = 0, hi = NAVLINK_MSG_COUNT;")
    p("    while (lo < hi) {")
    p("        size_t mid = (lo + hi) / 2;")
    p("        if (navlink_msg_table[mid].msgid == msgid) return &navlink_msg_table[mid];")
    p("        if (navlink_msg_table[mid].msgid < msgid) lo = mid + 1; else hi = mid;")
    p("    }")
    p("    return NULL;")
    p("}")
    p("")
    p(gen_c_frame_defs(d))
    return "\n".join(L) + "\n"


def gen_c_frame_decls(d):
    L = []
    p = L.append
    p("/* ===== framing: encode + incremental parser (spec §3–§4) ===== */")
    p("#define NAVLINK_SYNC        0x56u")
    p("#define NAVLINK_VERSION     0x02u")
    p("#define NAVLINK_HEADER_LEN  10u")
    p("#define NAVLINK_MAX_FRAME   267u   /* 10 header + 255 payload + 2 CRC */")
    p("")
    p("typedef struct { uint32_t msgid; uint8_t seq, sysid, compid, incompat_flags; } navlink_frame_hdr_t;")
    p("")
    p("/* Build a full frame (header + payload + CRC) from a packed payload; returns length. */")
    p("size_t navlink_frame(uint8_t *out, uint32_t msgid, const uint8_t *payload, size_t paylen,")
    p("                     uint8_t seq, uint8_t sysid, uint8_t compid);")
    p("")
    p("/* Per-message encoders: aligned struct -> full frame in `out` (>= NAVLINK_MAX_FRAME). */")
    for m in d["messages"]:
        pre = c_msg_prefix(m["name"])
        p(f"size_t {pre}_encode(uint8_t *out, const {pre}_t *msg, uint8_t seq, uint8_t sysid, uint8_t compid);")
    p("")
    p("/* Result a command handler returns; the dispatch packs it into the")
    p(" * COMMAND_ACK it emits on the handler's behalf (spec §12.1). Set `deferred`")
    p(" * (via navlink_ack_deferred()) to suppress the auto-send and emit the ack")
    p(" * yourself later — once async work finishes — with navlink_command_ack_send(). */")
    p("typedef struct { uint8_t result; uint8_t progress; int32_t result_param2; uint8_t deferred; } navlink_ack_t;")
    p("/* Immediate ack with `result` (dispatch sends it). */")
    p("static inline navlink_ack_t navlink_ack_result(uint8_t result) {")
    p("    navlink_ack_t a; a.result = result; a.progress = 0; a.result_param2 = 0; a.deferred = 0; return a; }")
    p("/* Defer: the handler owns the ack and must navlink_command_ack_send() it later. */")
    p("static inline navlink_ack_t navlink_ack_deferred(void) {")
    p("    navlink_ack_t a; a.result = 0; a.progress = 0; a.result_param2 = 0; a.deferred = 1; return a; }")
    p("")
    p("/* Populate the slots you care about; the parser fires one per decoded message.")
    p(" * Messages that require an ack (NAVLINK_ACK_<NAME>==1) hand their handler a")
    p(" * navlink_ack_t RESULT and the dispatch auto-sends the COMMAND_ACK via `send`")
    p(" * — a consumer chooses the result but cannot skip the ack. Set `send` (and")
    p(" * sysid/compid) whenever any ack-requiring handler is wired. */")
    p("typedef struct navlink_handlers {")
    p("    void *ctx;")
    p("    void (*send)(void *ctx, const uint8_t *frame, uint16_t len); /* emits COMMAND_ACK */")
    p("    uint8_t sysid, compid;  /* identity stamped on emitted ACKs */")
    for m in d["messages"]:
        pre = c_msg_prefix(m["name"])
        if requires_ack(m):
            p(f"    navlink_ack_t (*on_{m['name'].lower()})(void *ctx, const navlink_frame_hdr_t *hdr, const {pre}_t *msg); /* MUST return its COMMAND_ACK result */")
        else:
            p(f"    void (*on_{m['name'].lower()})(void *ctx, const navlink_frame_hdr_t *hdr, const {pre}_t *msg);")
    p("    /* Fallback for any decoded message whose specific on_<msg> slot is NULL. */")
    p("    void (*on_default)(void *ctx, const navlink_frame_hdr_t *hdr, uint32_t msgid, const uint8_t *payload, size_t len);")
    p("    void (*on_unknown)(void *ctx, uint32_t msgid, const uint8_t *payload, size_t len);")
    p("    void (*on_crc_error)(void *ctx, uint32_t msgid);")
    p("} navlink_handlers_t;")
    p("")
    p("typedef struct { uint8_t buf[NAVLINK_MAX_FRAME]; uint16_t idx, need; uint8_t state; } navlink_parser_t;")
    p("void navlink_parser_init(navlink_parser_t *p);")
    p("/* Feed received bytes; fires a handler for each complete, CRC-valid frame. */")
    p("void navlink_parser_push(navlink_parser_t *p, const navlink_handlers_t *h, const uint8_t *data, size_t n);")
    p("/* Build + send a COMMAND_ACK via h->send. The dispatch calls this automatically")
    p(" * for ack-requiring messages unless the handler returned navlink_ack_deferred();")
    p(" * a deferred handler calls it itself once the result is known. */")
    p("void navlink_command_ack_send(const navlink_handlers_t *h, uint32_t command, uint8_t req_seq, navlink_ack_t a);")
    return "\n".join(L)


def gen_c_frame_defs(d):
    L = []
    p = L.append
    p("size_t navlink_frame(uint8_t *out, uint32_t msgid, const uint8_t *payload, size_t paylen,")
    p("                     uint8_t seq, uint8_t sysid, uint8_t compid) {")
    p("    while (paylen && payload[paylen - 1] == 0) paylen--;   /* trailing-zero truncation §5.6 */")
    p("    const navlink_msg_info_t *mi = navlink_msg_info(msgid);")
    p("    uint8_t ce = mi ? mi->crc_extra : 0;")
    p("    out[0] = NAVLINK_SYNC; out[1] = NAVLINK_VERSION; out[2] = (uint8_t)paylen; out[3] = 0;")
    p("    out[4] = seq; out[5] = sysid; out[6] = compid;")
    p("    out[7] = (uint8_t)msgid; out[8] = (uint8_t)(msgid >> 8); out[9] = (uint8_t)(msgid >> 16);")
    p("    if (paylen) memcpy(out + NAVLINK_HEADER_LEN, payload, paylen);")
    p("    uint16_t crc = 0xFFFF;")
    p("    for (size_t i = 1; i < NAVLINK_HEADER_LEN; i++) navlink_crc_accumulate(out[i], &crc);")
    p("    for (size_t i = 0; i < paylen; i++) navlink_crc_accumulate(payload[i], &crc);")
    p("    navlink_crc_accumulate(ce, &crc);")
    p("    out[NAVLINK_HEADER_LEN + paylen] = (uint8_t)(crc & 0xFF);")
    p("    out[NAVLINK_HEADER_LEN + paylen + 1] = (uint8_t)(crc >> 8);")
    p("    return NAVLINK_HEADER_LEN + paylen + 2;")
    p("}")
    p("")
    for m in d["messages"]:
        pre = c_msg_prefix(m["name"])
        p(f"size_t {pre}_encode(uint8_t *out, const {pre}_t *msg, uint8_t seq, uint8_t sysid, uint8_t compid) {{")
        p(f"    {pre}_wire_t w; {pre}_from_aligned(&w, msg);")
        p(f"    uint8_t pay[{wire_size(m)}];")
        p(f"    size_t n = {pre}_pack(pay, &w);")
        p(f"    return navlink_frame(out, NAVLINK_MSGID_{m['name']}, pay, n, seq, sysid, compid);")
        p("}")
    p("")
    p("/* Build the COMMAND_ACK an ack-requiring message owes and hand it to")
    p(" * h->send. The dispatch calls this for every such message (unless the")
    p(" * handler deferred), so a consumer cannot silently drop the ack — it only")
    p(" * supplies the result (or, with no handler, the dispatch reports UNSUPPORTED). */")
    p("void navlink_command_ack_send(const navlink_handlers_t *h, uint32_t command,")
    p("                              uint8_t req_seq, navlink_ack_t a) {")
    p("    if (!h->send) return;")
    p("    navlink_command_ack_t m;")
    p("    m.command = command; m.req_seq = req_seq;")
    p("    m.result = a.result; m.progress = a.progress; m.result_param2 = a.result_param2;")
    p("    uint8_t frame[NAVLINK_MAX_FRAME];")
    p("    size_t n = navlink_command_ack_encode(frame, &m, req_seq, h->sysid, h->compid);")
    p("    h->send(h->ctx, frame, (uint16_t)n);")
    p("}")
    p("")
    p("static void navlink_dispatch(const navlink_frame_hdr_t *hdr, const uint8_t *pay, size_t len,")
    p("                             const navlink_handlers_t *h) {")
    p("    switch (hdr->msgid) {")
    for m in d["messages"]:
        pre, nm = c_msg_prefix(m["name"]), m["name"].lower()
        if requires_ack(m):
            # Always unpack (need req_seq), call the handler for its result, and
            # auto-emit the COMMAND_ACK. UNSUPPORTED when no handler is wired.
            p(f"    case NAVLINK_MSGID_{m['name']}: {{")
            p(f"        {pre}_wire_t w; {pre}_unpack(&w, pay, len);")
            p(f"        {pre}_t a; {pre}_to_aligned(&a, &w);")
            p("        navlink_ack_t _ack;")
            p(f"        if (h->on_{nm}) {{ _ack = h->on_{nm}(h->ctx, hdr, &a); }}")
            p("        else {")
            p("            _ack.result = NAVLINK_COMMAND_RESULT_UNSUPPORTED; _ack.progress = 0; _ack.result_param2 = 0; _ack.deferred = 0;")
            p(f"            if (h->on_default) h->on_default(h->ctx, hdr, NAVLINK_MSGID_{m['name']}, pay, len);")
            p("        }")
            p("        /* deferred handlers own the ack and send it later themselves */")
            p(f"        if (!_ack.deferred) navlink_command_ack_send(h, NAVLINK_MSGID_{m['name']}, a.req_seq, _ack);")
            p("        break;")
            p("    }")
        else:
            p(f"    case NAVLINK_MSGID_{m['name']}:")
            p(f"        if (h->on_{nm}) {{")
            p(f"            {pre}_wire_t w; {pre}_unpack(&w, pay, len);")
            p(f"            {pre}_t a; {pre}_to_aligned(&a, &w); h->on_{nm}(h->ctx, hdr, &a);")
            p("        } else if (h->on_default) {")
            p(f"            h->on_default(h->ctx, hdr, NAVLINK_MSGID_{m['name']}, pay, len);")
            p("        }")
            p("        break;")
    p("    default: break;")
    p("    }")
    p("}")
    p("")
    p("void navlink_parser_init(navlink_parser_t *p) { p->idx = 0; p->need = 0; p->state = 0; }")
    p("")
    p("static void navlink_parser_byte(navlink_parser_t *p, const navlink_handlers_t *h, uint8_t b) {")
    p("    if (p->state == 0) {                       /* hunting for sync */")
    p("        if (b == NAVLINK_SYNC) { p->buf[0] = b; p->idx = 1; p->state = 1; }")
    p("    } else if (p->state == 1) {                /* header */")
    p("        p->buf[p->idx++] = b;")
    p("        if (p->idx == NAVLINK_HEADER_LEN) {")
    p("            if (p->buf[1] != NAVLINK_VERSION) { p->state = 0; p->idx = 0; return; }")
    p("            p->need = (uint16_t)(NAVLINK_HEADER_LEN + p->buf[2] + 2u);")
    p("            p->state = 2;")
    p("        }")
    p("    } else {                                   /* payload + CRC */")
    p("        p->buf[p->idx++] = b;")
    p("        if (p->idx >= p->need) {")
    p("            uint8_t plen = p->buf[2];")
    p("            const uint8_t *pay = p->buf + NAVLINK_HEADER_LEN;")
    p("            navlink_frame_hdr_t hdr = { .incompat_flags = p->buf[3], .seq = p->buf[4],")
    p("                .sysid = p->buf[5], .compid = p->buf[6],")
    p("                .msgid = (uint32_t)p->buf[7] | ((uint32_t)p->buf[8] << 8) | ((uint32_t)p->buf[9] << 16) };")
    p("            const navlink_msg_info_t *mi = navlink_msg_info(hdr.msgid);")
    p("            if (!mi) {")
    p("                if (h->on_unknown) h->on_unknown(h->ctx, hdr.msgid, pay, plen);")
    p("            } else {")
    p("                uint16_t crc = 0xFFFF;")
    p("                for (uint8_t i = 1; i < NAVLINK_HEADER_LEN; i++) navlink_crc_accumulate(p->buf[i], &crc);")
    p("                for (uint8_t i = 0; i < plen; i++) navlink_crc_accumulate(pay[i], &crc);")
    p("                navlink_crc_accumulate(mi->crc_extra, &crc);")
    p("                uint16_t rx = (uint16_t)pay[plen] | ((uint16_t)pay[plen + 1] << 8);")
    p("                if (crc != rx) { if (h->on_crc_error) h->on_crc_error(h->ctx, hdr.msgid); }")
    p("                else navlink_dispatch(&hdr, pay, plen, h);")
    p("            }")
    p("            p->state = 0; p->idx = 0;")
    p("        }")
    p("    }")
    p("}")
    p("")
    p("void navlink_parser_push(navlink_parser_t *p, const navlink_handlers_t *h, const uint8_t *data, size_t n) {")
    p("    for (size_t i = 0; i < n; i++) navlink_parser_byte(p, h, data[i]);")
    p("}")
    return "\n".join(L)


def gen_c_parity(d):
    """Emit navlink_emit_parity(): fills each message with canonical_values() and
    prints `BYTES <NAME> <hex>`. The literals come from the same Python function
    the tests use, so C↔Python byte parity covers every message."""
    L = []
    p = L.append
    p("/* GENERATED by navlink/generate.py — DO NOT EDIT. Cross-language parity emitter. */")
    p('#include "navlink_msgs.h"')
    p("#include <stdio.h>")
    p("#include <string.h>")
    p("void navlink_emit_parity(void);")
    p("static void emitk(const char *kind, const char *name, const uint8_t *b, size_t n) {")
    p('    printf("%s %s ", kind, name);')
    p('    for (size_t i = 0; i < n; i++) printf("%02x", b[i]);')
    p('    printf("\\n");')
    p("}")
    p("void navlink_emit_parity(void) {")
    p("    uint8_t buf[512];")
    for m in d["messages"]:
        pre = c_msg_prefix(m["name"])
        p("    {")
        p(f"        {pre}_t a; memset(&a, 0, sizeof a);")
        for f, v in zip(ordered_fields(m), canonical_values(m)):
            nm, t, ln = f["name"], f["type"], field_len(f)
            if t == "char":
                for j, ch in enumerate(v):
                    p(f"        a.{nm}[{j}] = '{ch}';")
            elif ln is not None:
                for j, el in enumerate(v):
                    p(f"        a.{nm}[{j}] = {c_literal(t, el)};")
            else:
                p(f"        a.{nm} = {c_literal(t, v)};")
        p(f"        {pre}_wire_t w; {pre}_from_aligned(&w, &a);")
        p(f'        emitk("BYTES", "{m["name"]}", buf, {pre}_pack(buf, &w));')
        p("        uint8_t fr[NAVLINK_MAX_FRAME];")
        p(f'        emitk("FRAME", "{m["name"]}", fr, {pre}_encode(fr, &a, 7, 1, 1));')
        p("    }")
    p("}")
    return "\n".join(L) + "\n"


# ── Python generation ──────────────────────────────────────────────────────────
PY_PREAMBLE = '''\
# GENERATED by navlink/generate.py — DO NOT EDIT. Source: dialect.json
"""NavLink v2 message codecs (generated)."""
from __future__ import annotations
import collections
import struct
from dataclasses import dataclass, field
from enum import IntEnum

_SCALAR = {
    "u8": "B", "i8": "b", "u16": "H", "i16": "h", "u32": "I", "i32": "i",
    "u64": "Q", "i64": "q", "f32": "f", "f64": "d",
}


def crc_accumulate(b: int, crc: int) -> int:
    t = b ^ (crc & 0xFF)
    t = (t ^ (t << 4)) & 0xFF
    return ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc = crc_accumulate(b, crc)
    return crc


def _pack_field(ftype, flen, val):
    if ftype == "u24":
        return int(val).to_bytes(3, "little")
    if ftype == "char":
        b = val.encode() if isinstance(val, str) else bytes(val)
        return b[:flen].ljust(flen, b"\\x00")
    sc = _SCALAR[ftype]
    if flen is None:
        return struct.pack("<" + sc, val)
    return b"".join(struct.pack("<" + sc, v) for v in val)


def _unpack_field(ftype, flen, buf, off):
    if ftype == "u24":
        return int.from_bytes(buf[off:off + 3], "little"), off + 3
    if ftype == "char":
        raw = buf[off:off + flen]
        return raw.split(b"\\x00", 1)[0].decode("utf-8", "replace"), off + flen
    sc = _SCALAR[ftype]
    sz = struct.calcsize(sc)
    if flen is None:
        return struct.unpack_from("<" + sc, buf, off)[0], off + sz
    vals = list(struct.unpack_from("<" + sc * flen, buf, off))
    return vals, off + sz * flen
'''


def py_default(f):
    t, ln = f["type"], field_len(f)
    if t == "char":
        return '""'
    if ln is not None:
        zero = "0.0" if t in ("f32", "f64") else "0"
        return f"field(default_factory=lambda: [{', '.join([zero]*ln)}])"
    return "0.0" if t in ("f32", "f64") else "0"


def gen_py(d):
    L = []
    p = L.append
    p(PY_PREAMBLE)
    p("")
    # An enum whose PascalCase matches a message name is suffixed "Enum" so the
    # message dataclass (primary API) keeps the clean name.
    msg_pascal = {pascal(m["name"]) for m in d["messages"]}
    for ename, e in d.get("enums", {}).items():
        cls = pascal(ename)
        if cls in msg_pascal:
            cls += "Enum"
        p(f"class {cls}(IntEnum):")
        if e.get("doc"):
            p(f'    """{e["doc"]}"""')
        for ent in e["entries"]:
            p(f"    {ent['name']} = {ent['value']}")
        p("")
    p("")
    for m in d["messages"]:
        of = ordered_fields(m)
        p("@dataclass")
        p(f"class {pascal(m['name'])}:")
        if m.get("doc"):
            p(f'    """{m["doc"]}"""')
        p(f"    MSGID = {m['msgid']}")
        p(f"    CRC_EXTRA = {crc_extra(m)}")
        p(f"    WIRE_SIZE = {wire_size(m)}")
        p(f"    REQUIRES_ACK = {requires_ack(m)}")
        # field tuples (name, type, len) in wire order
        ftuples = ", ".join(f'("{f["name"]}", "{f["type"]}", {field_len(f)})' for f in of)
        p(f"    _FIELDS = [{ftuples}]")
        for f in of:
            p(f"    {f['name']}: object = {py_default(f)}")
        p("")
        p("    def pack(self) -> bytes:")
        p("        out = b''")
        p("        for nm, ft, ln in self._FIELDS:")
        p("            out += _pack_field(ft, ln, getattr(self, nm))")
        p("        return out")
        p("")
        p("    @classmethod")
        p("    def unpack(cls, buf: bytes):")
        p("        obj = cls(); off = 0")
        p("        buf = bytes(buf).ljust(cls.WIRE_SIZE, b'\\x00')")
        p("        for nm, ft, ln in cls._FIELDS:")
        p("            val, off = _unpack_field(ft, ln, buf, off)")
        p("            setattr(obj, nm, val)")
        p("        return obj")
        p("")
    # registries
    p("MSGID_TO_CLASS = {")
    for m in d["messages"]:
        p(f"    {m['msgid']}: {pascal(m['name'])},")
    p("}")
    p("CRC_EXTRA = {")
    for m in d["messages"]:
        p(f"    {m['msgid']}: {crc_extra(m)},")
    p("}")
    p("MSGID_TO_HANDLER = {")
    for m in d["messages"]:
        p(f"    {m['msgid']}: \"on_{m['name'].lower()}\",")
    p("}")
    p("")
    p(gen_py_frame(d))
    return "\n".join(L) + "\n"


def gen_py_frame(d):
    """Encoder + incremental Parser + per-message Handlers (mirrors the C frame layer)."""
    L = []
    p = L.append
    p("NAVLINK_SYNC = 0x56")
    p("NAVLINK_VERSION = 0x02")
    p("HEADER_LEN = 10")
    p("MAX_FRAME = 267")
    p("")
    p('Frame = collections.namedtuple("Frame", "msgid seq sysid compid incompat_flags payload")')
    p("")
    p("")
    p("def encode(msg, seq=0, sysid=1, compid=1):")
    p('    """Any generated message instance -> full frame bytes (header + payload + CRC)."""')
    p("    payload = msg.pack().rstrip(b\"\\x00\")        # trailing-zero truncation (§5.6)")
    p("    msgid = msg.MSGID")
    p("    hdr = bytes([NAVLINK_SYNC, NAVLINK_VERSION, len(payload), 0, seq & 0xFF,")
    p("                 sysid & 0xFF, compid & 0xFF,")
    p("                 msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF])")
    p("    crc = crc_accumulate(msg.CRC_EXTRA, crc16(hdr[1:] + payload))")
    p('    return hdr + payload + struct.pack("<H", crc)')
    p("")
    p("")
    p("@dataclass")
    p("class Handlers:")
    p('    """Populate the callbacks you care about; the Parser fires on_<message>(frame, msg)."""')
    for m in d["messages"]:
        p(f"    on_{m['name'].lower()}: object = None")
    p("    on_default: object = None      # on_default(frame, msg) — decoded msg w/o a specific handler")
    p("    on_unknown: object = None      # on_unknown(frame)")
    p("    on_crc_error: object = None    # on_crc_error(frame)")
    p("")
    p("")
    p("class Parser:")
    p('    """Incremental frame parser: feed bytes via push(), handlers fire per frame."""')
    p("    def __init__(self, handlers=None):")
    p("        self.h = handlers or Handlers()")
    p("        self._buf = bytearray()")
    p("        self._state = 0")
    p("        self._need = 0")
    p("")
    p("    def push(self, data):")
    p("        for b in bytes(data):")
    p("            self._byte(b)")
    p("")
    p("    def _byte(self, b):")
    p("        if self._state == 0:")
    p("            if b == NAVLINK_SYNC:")
    p("                self._buf = bytearray([b]); self._state = 1")
    p("        elif self._state == 1:")
    p("            self._buf.append(b)")
    p("            if len(self._buf) == HEADER_LEN:")
    p("                if self._buf[1] != NAVLINK_VERSION:")
    p("                    self._state = 0; self._buf = bytearray()")
    p("                else:")
    p("                    self._need = HEADER_LEN + self._buf[2] + 2")
    p("                    self._state = 2")
    p("        else:")
    p("            self._buf.append(b)")
    p("            if len(self._buf) >= self._need:")
    p("                self._handle()")
    p("                self._state = 0; self._buf = bytearray()")
    p("")
    p("    def _handle(self):")
    p("        buf = self._buf")
    p("        plen = buf[2]")
    p("        pay = bytes(buf[HEADER_LEN:HEADER_LEN + plen])")
    p("        msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16)")
    p("        frame = Frame(msgid, buf[4], buf[5], buf[6], buf[3], pay)")
    p("        ce = CRC_EXTRA.get(msgid)")
    p("        if ce is None:")
    p("            if self.h.on_unknown:")
    p("                self.h.on_unknown(frame)")
    p("            return")
    p("        crc = crc_accumulate(ce, crc16(bytes(buf[1:HEADER_LEN]) + pay))")
    p("        rx = buf[HEADER_LEN + plen] | (buf[HEADER_LEN + plen + 1] << 8)")
    p("        if crc != rx:")
    p("            if self.h.on_crc_error:")
    p("                self.h.on_crc_error(frame)")
    p("            return")
    p("        cb = getattr(self.h, MSGID_TO_HANDLER[msgid], None)")
    p("        msg = MSGID_TO_CLASS[msgid].unpack(pay)")
    p("        if cb:")
    p("            cb(frame, msg)")
    p("        elif self.h.on_default:")
    p("            self.h.on_default(frame, msg)")
    return "\n".join(L)


# ── driver ──────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="NavLink v2 code generator")
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--dialect", default=os.path.join(here, "dialect.json"))
    ap.add_argument("--out", default=os.path.join(here, "generated"))
    ap.add_argument("--lang", choices=["c", "py", "both"], default="both")
    args = ap.parse_args()

    with open(args.dialect) as fh:
        d = json.load(fh)

    errs = validate(d)
    if errs:
        print("dialect validation failed:", file=sys.stderr)
        for e in errs:
            print("  -", e, file=sys.stderr)
        sys.exit(1)

    # self-check (spec §16.1)
    assert crc16(b"123456789") == 0x6F91, "CRC-16/MCRF4XX self-check failed"

    os.makedirs(args.out, exist_ok=True)
    written = []
    if args.lang in ("c", "both"):
        cdir = os.path.join(args.out, "c")
        os.makedirs(cdir, exist_ok=True)
        with open(os.path.join(cdir, "navlink_msgs.h"), "w") as fh:
            fh.write(gen_c_header(d))
        with open(os.path.join(cdir, "navlink_msgs.c"), "w") as fh:
            fh.write(gen_c_source(d))
        with open(os.path.join(cdir, "navlink_parity.c"), "w") as fh:
            fh.write(gen_c_parity(d))
        written += [os.path.join(cdir, "navlink_msgs.h"), os.path.join(cdir, "navlink_msgs.c"),
                    os.path.join(cdir, "navlink_parity.c")]
    if args.lang in ("py", "both"):
        pdir = os.path.join(args.out, "python")
        os.makedirs(pdir, exist_ok=True)
        with open(os.path.join(pdir, "navlink_msgs.py"), "w") as fh:
            fh.write(gen_py(d))
        written += [os.path.join(pdir, "navlink_msgs.py")]

    print(f"navlink: {len(d['messages'])} messages, {len(d.get('enums', {}))} enums")
    print("CRC-16 self-check 0x{:04X} (expect 0x6F91) OK".format(crc16(b"123456789")))
    for w in written:
        print("  wrote", os.path.relpath(w, here))


if __name__ == "__main__":
    main()

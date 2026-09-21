#!/usr/bin/env python3
"""docs_html.py — render the tracked Markdown docs into a browsable HTML site.

Every Markdown file git tracks becomes an HTML page under the output directory,
mirroring its repo path, plus a generated index that groups the pages by the
per-component / four-layer taxonomy documented in firmware/docs/README.md.

pandoc does the Markdown -> HTML conversion. ```mermaid fences already come out
of pandoc as <pre class="mermaid">, so the diagrams are rendered client-side by
mermaid.js rather than pre-rasterised (contrast build_pdf.py, which shells out
to mmdc because LaTeX cannot run JavaScript).

Using git as the file oracle is deliberate: it excludes venvs, build trees and
the vendored extern/ submodule for free, and it means an untracked scratch file
never silently ships into the site.

Requires: pandoc. Mermaid is pulled from jsDelivr on first view, so diagrams
need network; all other content works offline. Browsers refuse cross-origin
module imports from file:// URLs, so use --serve (or any static server) if you
want the diagrams to draw.

Usage (from anywhere in the repo):
    tools/dev/docs_html.py                 # -> build_docs/
    tools/dev/docs_html.py --serve         # build, then serve it on :8000
    tools/dev/docs_html.py -o /tmp/site    # somewhere else
"""

import argparse
import html
import os
import re
import shutil
import subprocess
import tempfile
import sys
from pathlib import Path

MERMAID_CDN = "https://cdn.jsdelivr.net/npm/mermaid@11/dist/mermaid.esm.min.mjs"
# pandoc's bare --katex points at a Debian system path that no web server can
# resolve, so name the CDN base explicitly (pandoc appends katex.min.{js,css}).
KATEX_CDN = "https://cdn.jsdelivr.net/npm/katex@0.16/dist/"

# Index layout: (heading, path predicate). First match wins, so order matters.
COMPONENTS = [
    ("Start here", lambda p: "/" not in p),
    ("FC — firmware", lambda p: p.startswith("firmware/")),
    ("NavLink — wire protocol", lambda p: p.startswith("navlink/")),
    ("Sim", lambda p: p.startswith("sim/")),
    ("Tooling & tests", lambda p: True),
]

# The four lifecycle layers, in reading order (see firmware/docs/README.md).
LAYERS = ["reference", "plans", "journal", "scratch"]


def repo_root():
    out = subprocess.run(["git", "rev-parse", "--show-toplevel"],
                         capture_output=True, text=True, check=True)
    return Path(out.stdout.strip())


def tracked_md(root):
    out = subprocess.run(["git", "ls-files", "*.md"], cwd=root,
                         capture_output=True, text=True, check=True)
    return sorted(line for line in out.stdout.splitlines() if line)


def title_of(path, rel):
    """First ATX h1 wins; fall back to the filename so every page has a name."""
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            for line in f:
                if line.startswith("# "):
                    return line[2:].strip()
    except OSError:
        pass
    return Path(rel).stem


def layer_of(rel):
    for part in Path(rel).parts:
        if part in LAYERS:
            return part
    return None


# pandoc wraps every fenced block in <pre><code>. Harmless for real code, fatal
# for mermaid: mermaid's startOnLoad reads innerHTML (not textContent), so it
# would receive the literal "<code>" tag as the first token of the diagram and
# render a "Syntax error in text" bomb. Unwrap it — entity escaping stays,
# because mermaid entity-decodes what it reads.
MERMAID_PRE = re.compile(r'<pre class="mermaid"><code>(.*?)</code></pre>', re.S)


def postprocess(text, depth):
    """Fix up pandoc's HTML: unwrap mermaid blocks, repoint .md links at .html.

    Absolute URLs, anchors and non-Markdown targets (source files, images) are
    left alone — a link to firmware/src/foo.c is still a link to the source.
    """
    def sub(m):
        url = m.group(1)
        if re.match(r"^[a-z][a-z0-9+.-]*:", url, re.I) or url.startswith(("#", "/")):
            return m.group(0)
        target, sep, frag = url.partition("#")
        if not target.endswith(".md"):
            return m.group(0)
        return 'href="%s.html%s%s"' % (target[:-3], sep, frag)

    text = MERMAID_PRE.sub(lambda m: '<pre class="mermaid">%s</pre>' % m.group(1), text)
    return re.sub(r'href="([^"]*)"', sub, text)


TEMPLATE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>$title$ — Vayu docs</title>
<link rel="stylesheet" href="$root$assets/style.css">
$math$
</head>
<body>
<header class="topbar">
  <a class="home" href="$root$index.html">Vayu docs</a>
  <span class="crumb">$srcpath$</span>
</header>
<main>
$if(toc)$
<nav class="toc"><p class="toc-title">On this page</p>
$toc$
</nav>
$endif$
<article>
$body$
</article>
</main>
<dialog id="lightbox">
  <div class="lb-viewport"><div class="lb-stage"></div></div>
  <button class="lb-close" aria-label="Close">&times;</button>
  <p class="lb-hint">scroll to zoom &middot; drag to pan &middot; double-click to reset &middot; Esc to close</p>
</dialog>
<script>
(function () {
  var dlg = document.getElementById("lightbox");
  var vp    = dlg.querySelector(".lb-viewport");
  var stage = dlg.querySelector(".lb-stage");
  var s = 1, tx = 0, ty = 0, drag = null, touched = false;

  function apply() {
    stage.style.transform = "translate(" + tx + "px," + ty + "px) scale(" + s + ")";
  }
  // Open at the scale that fills the viewport WIDTH. "Contain" alone leaves a
  // tall portrait flowchart at ~45% width with unreadable labels, even though it
  // technically fits. Filling width and letting the reader pan down is what you
  // actually want from a diagram; wide content is unaffected (fit scale ~1).
  function fit(tries) {
    var el = stage.firstElementChild;
    if (!el) return;
    // Measured off documentElement, never off the dialog. The dialog is
    // 100vw x 100vh, so these ARE its dimensions — and unlike its own box they
    // are correct before it has been laid out. Measuring the dialog was racy:
    // it could report a right width with a stale height, and the min() below
    // then picked the width ratio and computed exactly 1x.
    var de = document.documentElement;
    var vr = { width: de.clientWidth, height: de.clientHeight };
    var cw, ch, k;
    if (vr.width > 0 && vr.height > 0) {
      if (el.tagName === "IMG") {
        if (el.naturalWidth) {
          k = Math.min(vr.width / el.naturalWidth, vr.height / el.naturalHeight);
          cw = el.naturalWidth * k; ch = el.naturalHeight * k;
        }
      } else {
        // Contained size comes from the viewBox, which is exactly what
        // preserveAspectRatio fits and is readable without waiting on layout.
        var vb = el.viewBox && el.viewBox.baseVal;
        if (vb && vb.width && vb.height) {
          k = Math.min(vr.width / vb.width, vr.height / vb.height);
          cw = vb.width * k; ch = vb.height * k;
        }
      }
    }
    if (!cw || !ch) {
      // The dialog is not laid out yet (or the image has not decoded), so the
      // viewport still measures 0. Retry next frame instead of silently
      // leaving the diagram at 1x — that race is why tall diagrams sometimes
      // opened unzoomed.
      if (tries > 0) requestAnimationFrame(function () { fit(tries - 1); });
      return;
    }
    s = Math.min(6, Math.max(1, vr.width / cw));
    tx = 0;
    // Anchor the top edge, so a long flowchart opens at its start, not its middle.
    ty = Math.max(0, (ch * s - vr.height) / 2);
    apply();
  }

  function reset() { s = 1; tx = 0; ty = 0; apply(); }
  function show(node) {
    stage.replaceChildren(node);
    touched = false;
    reset();
    dlg.showModal();
    // Synchronous: the fit reads documentElement, not the dialog, so there is
    // nothing to wait for. Deferring it to a rAF made the opening zoom depend on
    // a frame actually being scheduled, which is not guaranteed.
    fit(10);
    if (node.tagName === "IMG" && !node.complete) {
      node.addEventListener("load", function () { fit(10); });
    }
  }

  document.addEventListener("click", function (e) {
    var t = e.target;
    if (!t.closest || !t.closest("article")) return;

    // A rendered mermaid diagram is an inline <svg>, not an <img>. Clone it and
    // strip the width/height mermaid baked in, or it would open pinned to the
    // size it happened to have in the column. closest() works from an inner
    // <path>, so a click anywhere on the diagram counts.
    var pre = t.closest("pre.mermaid");
    if (pre) {
      var svg = pre.querySelector("svg");
      if (!svg) return;                      // not rendered yet
      var copy = svg.cloneNode(true);
      copy.removeAttribute("width");
      copy.removeAttribute("height");
      copy.style.maxWidth = "100%";
      copy.style.width = "100%";
      copy.style.height = "100%";
      show(copy);
      return;
    }

    // Never hijack an image that is itself a link.
    if (t.tagName !== "IMG" || t.closest("a")) return;
    var im = document.createElement("img");
    im.src = t.currentSrc || t.src;
    im.alt = t.alt || "";
    show(im);
  });

  // Re-fit on resize, but not once the reader has zoomed or panned — that would
  // throw away where they were looking.
  window.addEventListener("resize", function () {
    if (dlg.open && !touched) fit(0);
  });

  dlg.addEventListener("close", function () { stage.replaceChildren(); });
  dlg.querySelector(".lb-close").addEventListener("click", function () { dlg.close(); });
  dlg.addEventListener("click", function (e) { if (e.target === dlg) dlg.close(); });

  vp.addEventListener("wheel", function (e) {
    e.preventDefault();
    touched = true;
    var r  = vp.getBoundingClientRect();
    var mx = e.clientX - r.left - r.width  / 2;
    var my = e.clientY - r.top  - r.height / 2;
    var ns = Math.min(12, Math.max(1, s * (e.deltaY < 0 ? 1.15 : 1 / 1.15)));
    // Keep whatever sits under the cursor pinned there across the zoom.
    tx = mx - (mx - tx) * ns / s;
    ty = my - (my - ty) * ns / s;
    s = ns;
    if (s === 1) { tx = 0; ty = 0; }
    apply();
  }, { passive: false });

  vp.addEventListener("pointerdown", function (e) {
    touched = true;
    drag = { x: e.clientX - tx, y: e.clientY - ty };
    vp.classList.add("dragging");
    vp.setPointerCapture(e.pointerId);
  });
  vp.addEventListener("pointermove", function (e) {
    if (!drag) return;
    tx = e.clientX - drag.x; ty = e.clientY - drag.y; apply();
  });
  vp.addEventListener("pointerup", function () {
    drag = null; vp.classList.remove("dragging");
  });
  vp.addEventListener("dblclick", function () { touched = false; fit(0); });
})();
</script>
<script type="module">
  import mermaid from "MERMAID_URL";
  const dark = window.matchMedia("(prefers-color-scheme: dark)").matches;
  mermaid.initialize({ startOnLoad: true, theme: dark ? "dark" : "default" });
</script>
</body>
</html>
""".replace("MERMAID_URL", MERMAID_CDN)


STYLE = """/* Generated by tools/dev/docs_html.py — edit the script, not this file. */
:root {
  --bg: #ffffff; --fg: #1f2328; --muted: #59636e; --line: #d1d9e0;
  --accent: #0969da; --code-bg: #f6f8fa; --bar: #f6f8fa;
}
@media (prefers-color-scheme: dark) {
  :root {
    --bg: #0d1117; --fg: #e6edf3; --muted: #9198a1; --line: #3d444d;
    --accent: #4493f8; --code-bg: #161b22; --bar: #161b22;
  }
}
* { box-sizing: border-box; }
body {
  margin: 0; background: var(--bg); color: var(--fg);
  font: 16px/1.65 -apple-system, "Segoe UI", Roboto, "Helvetica Neue", sans-serif;
}
.topbar {
  position: sticky; top: 0; z-index: 5; display: flex; gap: 1rem; align-items: baseline;
  padding: .6rem 1.5rem; background: var(--bar); border-bottom: 1px solid var(--line);
}
.topbar .home { font-weight: 600; color: var(--fg); text-decoration: none; }
.topbar .crumb { color: var(--muted); font-family: ui-monospace, monospace;
                 font-size: .8rem; overflow-wrap: anywhere; }
.topbar .crumb a { color: var(--muted); text-decoration: none; }
.topbar .crumb a:hover { color: var(--accent); text-decoration: underline; }
.topbar .crumb .sep { opacity: .5; padding: 0 .1em; }
.topbar .crumb .here { color: var(--fg); }
main { display: flex; gap: 2.5rem; align-items: flex-start;
       max-width: 68rem; margin: 0 auto; padding: 2rem 1.5rem 6rem; }
article { min-width: 0; flex: 1; }
.toc {
  position: sticky; top: 3.4rem; flex: 0 0 15rem; order: 2;
  max-height: calc(100vh - 5rem); overflow-y: auto;
  font-size: .85rem; border-left: 1px solid var(--line); padding-left: 1rem;
}
.toc-title { margin: 0 0 .4rem; font-weight: 600; color: var(--muted);
             text-transform: uppercase; letter-spacing: .04em; font-size: .7rem; }
.toc ul { list-style: none; margin: 0; padding-left: .8rem; }
.toc > ul { padding-left: 0; }
.toc a { color: var(--muted); text-decoration: none; }
.toc a:hover { color: var(--accent); }
a { color: var(--accent); }
h1, h2, h3, h4 { line-height: 1.25; margin: 2rem 0 .8rem; }
h1 { font-size: 1.9rem; margin-top: 0; }
h2 { font-size: 1.4rem; padding-bottom: .3rem; border-bottom: 1px solid var(--line); }
h3 { font-size: 1.15rem; }
code { background: var(--code-bg); padding: .15em .35em; border-radius: 4px;
       font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-size: .875em; }
pre { background: var(--code-bg); padding: .9rem 1rem; border-radius: 6px;
      overflow-x: auto; border: 1px solid var(--line); }
pre code { background: none; padding: 0; }
pre.mermaid { background: none; border: none; text-align: center; cursor: zoom-in; }
/* Wide display math (e.g. the anti-saturation mix in firmware-control.md) scrolls
   in its own box rather than spilling out of the column — same contract as the
   tables and code blocks above. overflow-y stays hidden so the horizontal
   scrollbar cannot clip tall fractions. */
.katex-display { overflow-x: auto; overflow-y: hidden; padding: .4rem 0; }
blockquote { margin: 1rem 0; padding: .1rem 1rem; color: var(--muted);
             border-left: .25rem solid var(--line); }
table { border-collapse: collapse; display: block; overflow-x: auto; max-width: 100%; }
th, td { border: 1px solid var(--line); padding: .4rem .7rem; text-align: left; }
thead tr { background: var(--code-bg); }
article img { max-width: 100%; cursor: zoom-in; }
dialog#lightbox {
  position: fixed; inset: 0; margin: 0; border: 0; padding: 0;
  width: 100vw; height: 100vh; max-width: none; max-height: none;
  background: var(--bg); color: var(--fg);
}
dialog#lightbox::backdrop { background: rgba(0, 0, 0, .82); }
.lb-viewport {
  width: 100%; height: 100%; overflow: hidden; cursor: grab;
  display: flex; align-items: center; justify-content: center;
}
.lb-viewport.dragging { cursor: grabbing; }
.lb-stage {
  width: 100%; height: 100%; will-change: transform;
  display: flex; align-items: center; justify-content: center;
}
.lb-stage img, .lb-stage svg {
  /* Fill the stage on open rather than sitting at natural size: object-fit
     scales a small raster UP to the viewport (inline SVG ignores object-fit and
     uses its own preserveAspectRatio, which already does the same thing).
     Aspect ratio is preserved, so a wide-and-short plot still letterboxes
     vertically — that is the largest it can be without cropping. */
  width: 100%; height: 100%; object-fit: contain;
  user-select: none; -webkit-user-drag: none;
}
.lb-close {
  position: absolute; top: .4rem; right: .6rem; z-index: 1;
  background: var(--bar); color: var(--fg); border: 1px solid var(--line);
  border-radius: 6px; font-size: 1.3rem; line-height: 1;
  padding: .1rem .5rem .25rem; cursor: pointer;
}
.lb-hint {
  position: absolute; bottom: .4rem; left: 0; right: 0; margin: 0;
  text-align: center; color: var(--muted); font-size: .75rem;
  pointer-events: none;
}
hr { border: 0; border-top: 1px solid var(--line); margin: 2rem 0; }
.layer { margin-top: .4rem; }
@media (max-width: 60rem) { .toc { display: none; } }
"""


# Only the token colours, so the dark override cannot drag in breezedark's own
# code-block surface and fight the page theme.
TOKEN_RULE = re.compile(r"^code span\.[a-zA-Z]+\s*\{[^}]*\}", re.M)


def highlight_css(style, work, tokens_only=False):
    """Pandoc's syntax palette for one highlight style.

    pandoc will only populate `highlighting-css` for a document that actually
    contains highlighted code, hence the throwaway snippet; the variable holds
    raw CSS with no <style> wrapper.
    """
    tmpl = work / "hl.tmpl"
    tmpl.write_text("$highlighting-css$", encoding="utf-8")
    src = work / "hl.md"
    src.write_text("```c\nint x = 1;\n```\n", encoding="utf-8")
    css = subprocess.run(
        ["pandoc", "-f", "gfm", "-t", "html5", "--standalone",
         "--template", str(tmpl), "--highlight-style", style,
         "--metadata", "title=x", str(src)],
        capture_output=True, text=True, check=True).stdout
    return "\n".join(TOKEN_RULE.findall(css)) if tokens_only else css.strip()


def breadcrumb(rel, prefix, readme_dirs):
    """Turn a doc's path into clickable segments.

    A segment links to that directory's README page when the directory has one —
    which, given the docs taxonomy, is nearly always (every component root and
    every lifecycle layer ships a README.md). Segments without a README stay
    plain text rather than becoming dead links.
    """
    parts = rel.split("/")
    bits = []
    for i, part in enumerate(parts[:-1]):
        d = "/".join(parts[:i + 1])
        if d in readme_dirs:
            bits.append('<a href="%s%s/README.html">%s</a>' % (prefix, d, part))
        else:
            bits.append(part)
    bits.append('<span class="here">%s</span>' % parts[-1])
    return '<span class="sep">/</span>'.join(bits)


def render(src_md, rel, out_html, template, root_prefix, title, srcpath):
    out_html.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "pandoc", "-f", "gfm", "-t", "html5", "--standalone",
        "--toc", "--toc-depth=3",
        # Without this pandoc degrades LaTeX to unicode lookalikes and dumps
        # anything it cannot approximate (\begin{bmatrix}, \frac) as raw $$..$$.
        "--katex=" + KATEX_CDN,
        "--template", str(template),
        "--metadata", "title=" + title,
        "--variable", "root=" + root_prefix,
        "--variable", "srcpath=" + srcpath,
        "-o", "-", str(src_md),
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        sys.stderr.write("pandoc failed on %s:\n%s\n" % (rel, res.stderr))
        return False
    out_html.write_text(postprocess(res.stdout, root_prefix), encoding="utf-8")
    return True


def build_index(files, titles):
    """Group every page by component, then by lifecycle layer, as Markdown."""
    buckets = {name: [] for name, _ in COMPONENTS}
    for rel in files:
        for name, match in COMPONENTS:
            if match(rel):
                buckets[name].append(rel)
                break

    out = ["# Vayu documentation", "",
           "Generated from the %d Markdown files tracked in the repo. Docs are "
           "organised per component and, within a component, into four lifecycle "
           "layers — see [the taxonomy](firmware/docs/README.html)." % len(files), ""]

    for name, _ in COMPONENTS:
        rels = buckets[name]
        if not rels:
            continue
        out += ["## " + name, ""]
        by_layer = {}
        for rel in rels:
            by_layer.setdefault(layer_of(rel), []).append(rel)
        for layer in LAYERS + [None]:
            group = by_layer.get(layer)
            if not group:
                continue
            if layer:
                out += ['<p class="layer"><strong>%s/</strong></p>' % layer, ""]
            for rel in sorted(group):
                out.append("- [%s](%s.html) — `%s`"
                           % (html.escape(titles[rel]), rel[:-3], rel))
            out.append("")
    return "\n".join(out)


class Progress:
    """One self-overwriting status line on a tty; silent when piped.

    Deliberately stdlib-only and tty-gated: a redirected build (CI, `| tee`)
    should produce clean logs, not a few hundred half-drawn bars.
    """

    BAR = 24

    def __init__(self, total, label, stream=sys.stderr):
        self.total, self.label, self.stream = max(total, 1), label, stream
        self.tty = stream.isatty()
        if not self.tty:
            print("docs_html: %s %d %s..." % ("processing", total, label))

    def step(self, n, item=""):
        if not self.tty:
            return
        frac = n / self.total
        filled = int(self.BAR * frac)
        head = "  [%s%s] %3d%%  %d/%d %s  " % (
            "=" * filled, " " * (self.BAR - filled),
            int(frac * 100), n, self.total, self.label)
        room = shutil.get_terminal_size((80, 24)).columns - len(head) - 1
        if room > 1 and len(item) > room:
            item = "..." + item[-(room - 3):]
        elif room <= 1:
            item = ""
        self.stream.write("\r\033[K" + head + item)
        self.stream.flush()

    def done(self):
        if self.tty:
            self.stream.write("\r\033[K")
            self.stream.flush()


# Which future repository each top-level docs tree belongs to, per
# firmware/docs/plans/repo-split-submodules.md. sim/ ships with the firmware
# (sim/host IS the firmware compiled for host, and sim/vsim is the physics it
# links), so it is not a component of its own here.
SPLIT_SIDE = {
    "firmware": "firmware",
    "sim": "firmware",
    "tools": "firmware",
    "navlink": "navlink",
    "vtest": "vtest",
}

# Repos that are vendored as a SUBMODULE rather than living beside us. A link
# into one of these keeps resolving after the split, because the submodule sits
# at the same path it does today -- so these crossings are not a bill.
SUBMODULE_SIDES = {"navlink", "vtest"}

# Sibling repos -- ones with no path to us at all -- are now zero: navigator
# left, and its 14 inbound links became URLs in the same commit. The check
# stays at 0 so a relative link to a sibling repo cannot creep back in; a
# reference to one belongs in a URL, which this never counts.
MAX_CROSS_REPO_LINKS = 0


def split_side(repo_relative_path):
    """Which future repo a docs path belongs to; None for root-level pages."""
    return SPLIT_SIDE.get(repo_relative_path.split("/")[0])


def check_cross_repo_links(root):
    """Doc-to-doc links that leave their component. (src, dst, page, raw) rows.

    Read from the markdown, not the generated HTML: this is a question about
    the sources, and it must be answerable without building the site.

    Root-level pages (ARCHITECTURE.md, README.md) are exempt -- they describe
    the whole stack by design, and where they land is its own decision.
    """
    files = subprocess.run(["git", "ls-files", "*.md"], cwd=root,
                           capture_output=True, text=True,
                           check=True).stdout.split()
    rows = []
    for rel in files:
        src = split_side(rel)
        if src is None:
            continue
        try:
            text = (Path(root) / rel).read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        for m in re.finditer(r"\]\(([^)#][^)]*)\)", text):
            raw = m.group(1).split("#")[0].strip()
            if not raw or raw.startswith(("http://", "https://", "mailto:")):
                continue
            target = os.path.normpath(os.path.join(os.path.dirname(rel), raw))
            dst = split_side(target)
            if dst is None or dst == src or dst in SUBMODULE_SIDES:
                continue
            rows.append((src, dst, rel, raw))
    return rows


def check_links(out):
    """Report every doc-to-doc link that resolves to no generated page.

    Only .html targets are checked: a link to firmware/src/foo.c is a pointer at
    the source tree, not a broken page, and is left alone.
    """
    bad = []
    for page in sorted(out.rglob("*.html")):
        for url in re.findall(r'href="([^"]*)"', page.read_text(encoding="utf-8",
                                                                 errors="replace")):
            if re.match(r"^[a-z][a-z0-9+.-]*:", url, re.I) or url.startswith("#"):
                continue
            target = url.split("#")[0]
            if target.endswith(".html") and not (page.parent / target).exists():
                bad.append((page.relative_to(out), url))
    return bad


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("-o", "--out", default="build_docs", help="output dir (default: build_docs)")
    ap.add_argument("--check", action="store_true",
                    help="after building, fail if any doc-to-doc link dangles")
    ap.add_argument("--serve", action="store_true", help="serve the site after building")
    ap.add_argument("--port", type=int, default=8000)
    args = ap.parse_args()

    if not shutil.which("pandoc"):
        sys.exit("docs_html: pandoc not found on PATH (apt install pandoc)")

    root = repo_root()
    out = Path(args.out)
    if not out.is_absolute():
        out = root / out
    out.mkdir(parents=True, exist_ok=True)

    files = tracked_md(root)
    titles = {rel: title_of(root / rel, rel) for rel in files}
    # Computed up front, not by probing the output dir: pages are rendered in
    # sorted order, so a directory's README.html may not exist on disk yet when
    # a sibling page needs to link to it.
    readme_dirs = {rel.rsplit("/", 1)[0] for rel in files
                   if rel.endswith("/README.md")}

    (out / "assets").mkdir(exist_ok=True)

    # The pandoc template and the synthesised index source are build inputs, not
    # pages — keep them out of the published tree so --check never scans them.
    work = Path(tempfile.mkdtemp(prefix="docs_html."))
    template = work / "template.html"
    template.write_text(TEMPLATE, encoding="utf-8")

    # Syntax colours are baked into the shared sheet rather than inlined into
    # all 185 pages. pygments reads well on the light theme and carries no
    # background rules of its own; breezedark supplies the dark palette.
    (out / "assets" / "style.css").write_text(
        STYLE
        + "\n/* syntax highlighting (pandoc: pygments) */\n"
        + highlight_css("pygments", work)
        + "\n\n@media (prefers-color-scheme: dark) {\n"
        + "/* syntax highlighting (pandoc: breezedark), token colours only */\n"
        + highlight_css("breezedark", work, tokens_only=True)
        + "\n}\n", encoding="utf-8")

    ok = 0
    bar = Progress(len(files), "pages")
    for i, rel in enumerate(files, 1):
        bar.step(i, rel)
        prefix = "../" * rel.count("/")
        if render(root / rel, rel, out / (rel[:-3] + ".html"), template,
                  prefix, titles[rel], breadcrumb(rel, prefix, readme_dirs)):
            ok += 1
    bar.done()

    index_md = work / "index.md"
    index_md.write_text(build_index(files, titles), encoding="utf-8")
    render(index_md, "index.md", out / "index.html", template, "",
           "Vayu documentation", "%d pages" % len(files))

    # Images and other assets live beside their .md; copy what the docs reference.
    assets = subprocess.run(
        ["git", "ls-files", "*.png", "*.jpg", "*.jpeg", "*.svg", "*.gif", "*.pdf"],
        cwd=root, capture_output=True, text=True, check=True).stdout.split()
    bar = Progress(len(assets), "assets")
    copied = 0
    for i, rel in enumerate(assets, 1):
        bar.step(i, rel)
        dst = out / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(root / rel, dst)
        copied += 1
    bar.done()

    shutil.rmtree(work, ignore_errors=True)

    print("docs_html: %d/%d pages + %d assets -> %s"
          % (ok, len(files), copied, out))
    print("           open %s/index.html" % out)

    if args.check:
        # Boundary report first: it explains WHY a link that resolves today is
        # still a problem, which a plain broken-link list cannot.
        cross = check_cross_repo_links(root)
        if cross:
            print("docs_html: %d link(s) cross a SIBLING repository boundary "
                  "(ceiling %d)" % (len(cross), MAX_CROSS_REPO_LINKS),
                  file=sys.stderr)
            for src, dst, page, raw in sorted(cross):
                print("  %s -> %s  %s -> %s" % (src, dst, page, raw),
                      file=sys.stderr)
        if len(cross) > MAX_CROSS_REPO_LINKS:
            print("docs_html: cross-repo links rose to %d (ceiling %d). These "
                  "resolve now and will not once the repos split -- make it a "
                  "URL, or raise MAX_CROSS_REPO_LINKS deliberately."
                  % (len(cross), MAX_CROSS_REPO_LINKS), file=sys.stderr)
            return 1

        bad = check_links(out)
        for page, url in bad:
            print("  broken: %s -> %s" % (page, url), file=sys.stderr)
        print("docs_html: %d broken inter-doc link(s)" % len(bad),
              file=sys.stderr if bad else sys.stdout)
        if bad:
            return 1

    if args.serve:
        os.chdir(out)
        from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
        print("           serving on http://localhost:%d/ (ctrl-c to stop)" % args.port)
        ThreadingHTTPServer(("", args.port), SimpleHTTPRequestHandler).serve_forever()


if __name__ == "__main__":
    sys.exit(main() or 0)

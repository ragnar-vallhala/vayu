#include "PacketFilterExpr.h"

#include <QVector>

namespace {

enum class Field { Type, Dir, Dev, Size, Origin, Info, Invalid };
enum class Op { Eq, Ne, Lt, Le, Gt, Ge, Contains, Invalid };

struct Tok {
  enum Kind { AndOp, OrOp, NotOp, LParen, RParen, Cmp, Word, Num, Str, End } kind;
  QString text;
};

Field fieldOf(const QString &w) {
  const QString s = w.toLower();
  if (s == "type") return Field::Type;
  if (s == "dir") return Field::Dir;
  if (s == "dev") return Field::Dev;
  if (s == "size" || s == "len") return Field::Size;
  if (s == "origin") return Field::Origin;
  if (s == "info") return Field::Info;
  return Field::Invalid;
}

Op opOf(const QString &s) {
  if (s == "==") return Op::Eq;
  if (s == "!=") return Op::Ne;
  if (s == "<") return Op::Lt;
  if (s == "<=") return Op::Le;
  if (s == ">") return Op::Gt;
  if (s == ">=") return Op::Ge;
  if (s.compare("contains", Qt::CaseInsensitive) == 0) return Op::Contains;
  return Op::Invalid;
}

bool numCmp(long a, Op op, long b) {
  switch (op) {
  case Op::Eq: case Op::Contains: return a == b;
  case Op::Ne: return a != b;
  case Op::Lt: return a < b;
  case Op::Le: return a <= b;
  case Op::Gt: return a > b;
  case Op::Ge: return a >= b;
  default: return false;
  }
}

long parseNum(const QString &s, bool *ok) {
  if (s.startsWith("0x", Qt::CaseInsensitive))
    return s.mid(2).toLong(ok, 16);
  return s.toLong(ok, 10);
}

QVector<Tok> lex(const QString &src, bool *ok) {
  QVector<Tok> out;
  *ok = true;
  int i = 0, n = src.size();
  auto at = [&](int k) { return k < n ? src[k] : QChar('\0'); };
  while (i < n) {
    QChar c = src[i];
    if (c.isSpace()) { ++i; continue; }
    if (c == '(') { out.push_back({Tok::LParen, "("}); ++i; continue; }
    if (c == ')') { out.push_back({Tok::RParen, ")"}); ++i; continue; }
    if (c == '&' && at(i + 1) == '&') { out.push_back({Tok::AndOp, "&&"}); i += 2; continue; }
    if (c == '|' && at(i + 1) == '|') { out.push_back({Tok::OrOp, "||"}); i += 2; continue; }
    if (c == '=' && at(i + 1) == '=') { out.push_back({Tok::Cmp, "=="}); i += 2; continue; }
    if (c == '!' && at(i + 1) == '=') { out.push_back({Tok::Cmp, "!="}); i += 2; continue; }
    if (c == '!') { out.push_back({Tok::NotOp, "!"}); ++i; continue; }
    if (c == '<') { if (at(i + 1) == '=') { out.push_back({Tok::Cmp, "<="}); i += 2; } else { out.push_back({Tok::Cmp, "<"}); ++i; } continue; }
    if (c == '>') { if (at(i + 1) == '=') { out.push_back({Tok::Cmp, ">="}); i += 2; } else { out.push_back({Tok::Cmp, ">"}); ++i; } continue; }
    if (c == '"') {
      int j = i + 1; QString s;
      while (j < n && src[j] != '"') s += src[j++];
      if (j >= n) { *ok = false; return out; }
      out.push_back({Tok::Str, s}); i = j + 1; continue;
    }
    if (c.isDigit()) {
      int j = i; while (j < n && (src[j].isLetterOrNumber())) ++j; // 0x.. too
      out.push_back({Tok::Num, src.mid(i, j - i)}); i = j; continue;
    }
    if (c.isLetter() || c == '_') {
      int j = i;
      while (j < n && (src[j].isLetterOrNumber() || src[j] == '_' || src[j] == '.')) ++j;
      out.push_back({Tok::Word, src.mid(i, j - i)}); i = j; continue;
    }
    *ok = false; return out; // unexpected char
  }
  out.push_back({Tok::End, ""});
  return out;
}

using Pred = std::function<bool(const FilterCtx &)>;

struct Parser {
  const QVector<Tok> &t;
  int p = 0;
  bool ok = true;
  const Tok &cur() const { return t[p]; }
  void eat() { if (t[p].kind != Tok::End) ++p; }

  Pred parse() { Pred e = orE(); if (cur().kind != Tok::End) ok = false; return e; }

  Pred orE() {
    Pred a = andE();
    while (ok && cur().kind == Tok::OrOp) {
      eat(); Pred b = andE();
      a = [a, b](const FilterCtx &c) { return a(c) || b(c); };
    }
    return a;
  }
  Pred andE() {
    Pred a = notE();
    while (ok && cur().kind == Tok::AndOp) {
      eat(); Pred b = notE();
      a = [a, b](const FilterCtx &c) { return a(c) && b(c); };
    }
    return a;
  }
  Pred notE() {
    if (cur().kind == Tok::NotOp) {
      eat(); Pred a = notE();
      return [a](const FilterCtx &c) { return !a(c); };
    }
    return atom();
  }
  Pred atom() {
    if (cur().kind == Tok::LParen) {
      eat(); Pred a = orE();
      if (cur().kind != Tok::RParen) { ok = false; return {}; }
      eat(); return a;
    }
    return comparison();
  }
  Pred comparison() {
    if (cur().kind != Tok::Word) { ok = false; return {}; }
    Field f = fieldOf(cur().text);
    if (f == Field::Invalid) { ok = false; return {}; }
    eat();
    Op op;
    if (cur().kind == Tok::Cmp) op = opOf(cur().text);
    else if (cur().kind == Tok::Word && opOf(cur().text) == Op::Contains) op = Op::Contains;
    else { ok = false; return {}; }
    eat();
    if (cur().kind != Tok::Word && cur().kind != Tok::Num && cur().kind != Tok::Str) {
      ok = false; return {};
    }
    const QString val = cur().text;
    eat();
    bool numOk = false; long num = parseNum(val, &numOk);
    const QString low = val.toLower();
    return [f, op, low, num, numOk](const FilterCtx &c) -> bool {
      switch (f) {
      case Field::Type:
        if (numOk) return numCmp(c.type, op, num);
        else {
          bool m = c.typeName.contains(low);
          return op == Op::Ne ? !m : m; // Eq/Contains -> m
        }
      case Field::Dir: {
        if (low != "rx" && low != "tx") return false;
        bool m = (c.tx == (low == "tx"));
        return op == Op::Ne ? !m : m;
      }
      case Field::Dev: return numOk && numCmp(c.dev, op, num);
      case Field::Size: return numOk && numCmp(c.len, op, num);
      case Field::Origin:
        if (!numOk) return false;
        if (c.origin < 0) return op == Op::Ne;
        return numCmp(c.origin, op, num);
      case Field::Info: {
        bool m = c.info.contains(low);
        return op == Op::Ne ? !m : m;
      }
      default: return false;
      }
    };
  }
};

} // namespace

bool PacketFilterExpr::compile(const QString &src) {
  const QString s = src.trimmed();
  m_empty = s.isEmpty();
  m_valid = true;
  m_err.clear();
  m_pred = nullptr;
  if (m_empty)
    return true;

  bool lexOk = false;
  QVector<Tok> toks = lex(s, &lexOk);
  if (!lexOk) { m_valid = false; m_err = "lex error"; return false; }

  Parser pr{toks};
  Pred pred = pr.parse();
  if (!pr.ok || !pred) { m_valid = false; m_err = "syntax error"; return false; }
  m_pred = pred;
  return true;
}

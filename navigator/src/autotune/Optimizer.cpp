#include "Optimizer.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace autotune {

double Evaluator::eval(const Vec &x) {
  if (m_n >= m_budget)
    throw BudgetExhausted{};
  const double v = m_fn(x);
  ++m_n;
  if (v < m_best) {
    m_best = v;
    m_bestX = x;
    m_hasBest = true;
  }
  m_history.push_back({m_n, v, m_best});
  if (onEval)
    onEval(x, m_bestX, v, m_best, m_n);
  return v;
}

namespace {

Vec clampVec(const Vec &x, const Bounds &b) {
  Vec out(x.size());
  for (size_t i = 0; i < x.size(); ++i)
    out[i] = std::min(b[i].second, std::max(b[i].first, x[i]));
  return out;
}

Vec spans(const Bounds &b) {
  Vec s(b.size());
  for (size_t i = 0; i < b.size(); ++i)
    s[i] = b[i].second - b[i].first;
  return s;
}

// --- optimizers (ports of optimizers.py) ----------------------------------

void randomSearch(Evaluator &ev, const Vec &x0, const Bounds &b, Rng &rng) {
  try {
    ev.eval(clampVec(x0, b));
    while (true) {
      Vec x(b.size());
      for (size_t i = 0; i < b.size(); ++i)
        x[i] = rng.uniform(b[i].first, b[i].second);
      ev.eval(x);
    }
  } catch (const BudgetExhausted &) {
    /* Budget spent mid-strategy: stop here, keep the best so far. */
  }
}

void spsa(Evaluator &ev, const Vec &x0, const Bounds &b, Rng &rng) {
  const int n = int(x0.size());
  const Vec span = spans(b);
  const double a = 0.10, c = 0.08, A = 5.0, alpha = 0.602, gamma = 0.101;
  Vec x = clampVec(x0, b);
  try {
    int k = 0;
    while (true) {
      const double ak = a / std::pow(k + 1 + A, alpha);
      const double ck = c / std::pow(k + 1, gamma);
      Vec delta(n), xp(n), xm(n);
      for (int i = 0; i < n; ++i) {
        delta[i] = rng.coin() ? 1.0 : -1.0;
        xp[i] = x[i] + ck * span[i] * delta[i];
        xm[i] = x[i] - ck * span[i] * delta[i];
      }
      xp = clampVec(xp, b);
      xm = clampVec(xm, b);
      const double yp = ev.eval(xp);
      const double ym = ev.eval(xm);
      for (int i = 0; i < n; ++i) {
        const double g = (yp - ym) / (2.0 * ck * span[i] * delta[i]);
        x[i] = x[i] - ak * span[i] * span[i] * g;
      }
      x = clampVec(x, b);
      ++k;
    }
  } catch (const BudgetExhausted &) {
    /* Budget spent mid-strategy: stop here, keep the best so far. */
  }
}

void fdgd(Evaluator &ev, const Vec &x0, const Bounds &b, Rng &,
          double lr = 0.15, double eps = 0.05) {
  const int n = int(x0.size());
  const Vec span = spans(b);
  Vec x = clampVec(x0, b);
  try {
    while (true) {
      Vec g(n);
      for (int i = 0; i < n; ++i) {
        const double h = eps * span[i];
        Vec xp = x, xm = x;
        xp[i] = std::min(b[i].second, x[i] + h);
        xm[i] = std::max(b[i].first, x[i] - h);
        g[i] = (ev.eval(xp) - ev.eval(xm)) / (xp[i] - xm[i] + 1e-12);
      }
      Vec nx(n);
      for (int i = 0; i < n; ++i)
        nx[i] = x[i] - lr * span[i] * span[i] * g[i] / (span[i] + 1e-9);
      x = clampVec(nx, b);
    }
  } catch (const BudgetExhausted &) {
    /* Budget spent mid-strategy: stop here, keep the best so far. */
  }
}

void coordinate(Evaluator &ev, const Vec &x0, const Bounds &b, Rng &,
                double step = 0.25, double shrink = 0.5,
                double minStep = 0.02) {
  const int n = int(x0.size());
  const Vec span = spans(b);
  Vec x = clampVec(x0, b);
  try {
    double fx = ev.eval(x);
    double s = step;
    while (s >= minStep) {
      bool improved = false;
      for (int i = 0; i < n; ++i) {
        for (double sign : {1.0, -1.0}) {
          Vec cand = x;
          cand[i] = std::min(b[i].second,
                             std::max(b[i].first, x[i] + sign * s * span[i]));
          const double fc = ev.eval(cand);
          if (fc < fx) {
            x = cand;
            fx = fc;
            improved = true;
          }
        }
      }
      if (!improved)
        s *= shrink;
    }
  } catch (const BudgetExhausted &) {
    /* Budget spent mid-strategy: stop here, keep the best so far. */
  }
}

void nelderMead(Evaluator &ev, const Vec &x0, const Bounds &b, Rng &,
                double init = 0.15) {
  const int n = int(x0.size());
  const Vec span = spans(b);
  auto makeSimplex = [&](const Vec &center) {
    std::vector<Vec> S{clampVec(center, b)};
    for (int i = 0; i < n; ++i) {
      Vec p = center;
      p[i] = std::min(b[i].second, p[i] + init * span[i]);
      S.push_back(clampVec(p, b));
    }
    return S;
  };
  try {
    std::vector<Vec> simplex = makeSimplex(x0);
    std::vector<double> fs;
    for (auto &p : simplex)
      fs.push_back(ev.eval(p));
    while (true) {
      std::vector<int> order(n + 1);
      std::iota(order.begin(), order.end(), 0);
      std::sort(order.begin(), order.end(),
                [&](int a, int c) { return fs[a] < fs[c]; });
      std::vector<Vec> ns(n + 1);
      std::vector<double> nf(n + 1);
      for (int j = 0; j <= n; ++j) {
        ns[j] = simplex[order[j]];
        nf[j] = fs[order[j]];
      }
      simplex.swap(ns);
      fs.swap(nf);

      Vec centroid(n, 0.0);
      for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j)
          centroid[i] += simplex[j][i];
        centroid[i] /= n;
      }
      const Vec &worst = simplex[n];
      Vec refl(n);
      for (int i = 0; i < n; ++i)
        refl[i] = centroid[i] + 1.0 * (centroid[i] - worst[i]);
      refl = clampVec(refl, b);
      const double fr = ev.eval(refl);
      if (fr < fs[0]) {
        Vec exp(n);
        for (int i = 0; i < n; ++i)
          exp[i] = centroid[i] + 2.0 * (centroid[i] - worst[i]);
        exp = clampVec(exp, b);
        const double fe = ev.eval(exp);
        if (fe < fr) {
          simplex[n] = exp;
          fs[n] = fe;
        } else {
          simplex[n] = refl;
          fs[n] = fr;
        }
      } else if (fr < fs[n - 1]) {
        simplex[n] = refl;
        fs[n] = fr;
      } else {
        Vec con(n);
        for (int i = 0; i < n; ++i)
          con[i] = centroid[i] + 0.5 * (worst[i] - centroid[i]);
        con = clampVec(con, b);
        const double fc = ev.eval(con);
        if (fc < fs[n]) {
          simplex[n] = con;
          fs[n] = fc;
        } else {
          const Vec best = simplex[0];
          for (int j = 1; j <= n; ++j) {
            Vec p(n);
            for (int i = 0; i < n; ++i)
              p[i] = best[i] + 0.5 * (simplex[j][i] - best[i]);
            simplex[j] = clampVec(p, b);
            fs[j] = ev.eval(simplex[j]);
          }
        }
      }
      double spread = 0.0;
      for (int i = 0; i < n; ++i)
        spread = std::max(spread, std::fabs(simplex[0][i] - simplex[n][i]) /
                                      (span[i] + 1e-9));
      if (spread < 1e-3) {
        simplex = makeSimplex(simplex[0]);
        fs.clear();
        for (auto &p : simplex)
          fs.push_back(ev.eval(p));
      }
    }
  } catch (const BudgetExhausted &) {
    /* Budget spent mid-strategy: stop here, keep the best so far. */
  }
}

void structured(Evaluator &ev, const Vec &x0, const Bounds &b, Rng &) {
  const int n = int(x0.size());
  Vec x = clampVec(x0, b);
  std::vector<int> order;
  for (int i : {0, 2, 1, 3})
    if (i < n)
      order.push_back(i);
  if (n >= 9)
    for (int i : {5, 7, 6})
      if (i < n)
        order.push_back(i);
  for (int idx : {0, 1, 2})
    if (idx < n)
      x[idx] = b[idx].first;
  if (n >= 9)
    for (int idx : {5, 6, 7})
      x[idx] = b[idx].first;
  const int pts = std::max(4, ev.budget() / (int(order.size()) + 1));
  try {
    ev.eval(x);
    for (int idx : order) {
      const double lo = b[idx].first, hi = b[idx].second;
      double bestV = std::numeric_limits<double>::infinity();
      double bestVal = x[idx];
      for (int k = 0; k < pts; ++k) {
        x[idx] = lo + (hi - lo) * k / (pts - 1);
        const double v = ev.eval(x);
        if (v < bestV) {
          bestV = v;
          bestVal = x[idx];
        }
      }
      x[idx] = bestVal;
    }
  } catch (const BudgetExhausted &) {
    /* Budget spent mid-strategy: stop here, keep the best so far. */
  }
}

void hybrid(Evaluator &ev, const Vec &x0, const Bounds &b, Rng &rng,
            double exploreFrac = 0.35) {
  const int explore = std::max(2, int(ev.budget() * exploreFrac));
  try {
    ev.eval(clampVec(x0, b));
    for (int i = 0; i < explore - 1; ++i) {
      Vec x(b.size());
      for (size_t j = 0; j < b.size(); ++j)
        x[j] = rng.uniform(b[j].first, b[j].second);
      ev.eval(x);
    }
  } catch (const BudgetExhausted &) {
    return;
  }
  spsa(ev, ev.hasBest() ? ev.bestX() : x0, b, rng);
}

void portfolio(Evaluator &ev, const Vec &x0, const Bounds &b, Rng &rng);

} // namespace

void run(const std::string &name, Evaluator &ev, const Vec &x0,
         const Bounds &bounds, Rng &rng) {
  if (name == "spsa")
    spsa(ev, x0, bounds, rng);
  else if (name == "fdgd")
    fdgd(ev, x0, bounds, rng);
  else if (name == "coordinate")
    coordinate(ev, x0, bounds, rng);
  else if (name == "nelder-mead")
    nelderMead(ev, x0, bounds, rng);
  else if (name == "structured")
    structured(ev, x0, bounds, rng);
  else if (name == "hybrid")
    hybrid(ev, x0, bounds, rng);
  else if (name == "portfolio")
    portfolio(ev, x0, bounds, rng);
  else
    randomSearch(ev, x0, bounds, rng); // "random" + unknown
}

namespace {

void portfolio(Evaluator &ev, const Vec &x0, const Bounds &b, Rng &rng) {
  const std::vector<std::string> methods = {"spsa", "nelder-mead",
                                            "coordinate"};
  const int per = std::max(2, ev.budget() / int(methods.size()));
  for (const auto &m : methods) {
    if (ev.left() <= 0)
      break;
    const int cap = std::min(per, ev.left());
    // Sub-evaluator caps this method's slice while delegating to the global
    // ev (so the global best/history/budget keep updating).
    Evaluator sub([&ev](const Vec &x) { return ev.eval(x); }, cap);
    try {
      run(m, sub, x0, b, rng);
    } catch (const BudgetExhausted &) {
      /* Budget spent mid-strategy: stop here, keep the best so far. */
    }
  }
}

} // namespace

std::vector<std::string> optimizerNames() {
  return {"random",      "spsa",       "fdgd",   "coordinate",
          "nelder-mead", "structured", "hybrid", "portfolio"};
}

} // namespace autotune

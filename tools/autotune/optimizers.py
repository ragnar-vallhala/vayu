"""Derivative-free / estimated-gradient optimizers for PID autotuning.

Every optimizer minimizes a noisy black-box cost f(x) over box bounds, sharing
one Evaluator so a global evaluation budget is enforced and the best point is
tracked across methods. Pure stdlib (no numpy) to keep the harness portable.

Provided: random_search, spsa, fdgd (finite-difference gradient descent),
coordinate (pattern descent), nelder_mead, plus two meta-strategies:
  hybrid    -- coarse random exploration then SPSA local refine
  portfolio -- split the budget across several optimizers, keep the best
"""

import math
import random


class BudgetExhausted(Exception):
    pass


class Evaluator:
    """Wraps the cost fn: counts evals, tracks the running best, logs history."""

    def __init__(self, fn, budget):
        self.fn = fn
        self.budget = budget
        self.n = 0
        self.best = math.inf
        self.best_x = None
        self.history = []          # (eval_index, cost, running_best)

    def __call__(self, x):
        if self.n >= self.budget:
            raise BudgetExhausted()
        x = [float(v) for v in x]
        v = self.fn(x)
        self.n += 1
        if v < self.best:
            self.best, self.best_x = v, list(x)
        self.history.append((self.n, v, self.best))
        return v

    @property
    def left(self):
        return self.budget - self.n


def _clamp(x, bounds):
    return [min(hi, max(lo, v)) for v, (lo, hi) in zip(x, bounds)]


# ---------------------------------------------------------------------------
def random_search(ev, x0, bounds, rng):
    """Uniform random sampling over the box (seeded with x0 as first point)."""
    try:
        ev(_clamp(x0, bounds))
        while True:
            ev([rng.uniform(lo, hi) for lo, hi in bounds])
    except BudgetExhausted:
        pass


def spsa(ev, x0, bounds, rng, a=None, c=None):
    """Simultaneous Perturbation Stochastic Approximation: 2 evals per step,
    a stochastic gradient estimate that scales to any dimension and tolerates
    noisy costs. Step/perturbation sizes decay on the standard SPSA schedule."""
    n = len(x0)
    span = [hi - lo for lo, hi in bounds]
    a = a if a is not None else 0.10
    c = c if c is not None else 0.08
    A = 5.0
    alpha, gamma = 0.602, 0.101
    x = _clamp(list(x0), bounds)
    try:
        k = 0
        while True:
            ak = a / ((k + 1 + A) ** alpha)
            ck = c / ((k + 1) ** gamma)
            delta = [1.0 if rng.random() < 0.5 else -1.0 for _ in range(n)]
            xp = _clamp([x[i] + ck * span[i] * delta[i] for i in range(n)], bounds)
            xm = _clamp([x[i] - ck * span[i] * delta[i] for i in range(n)], bounds)
            yp, ym = ev(xp), ev(xm)
            for i in range(n):
                g = (yp - ym) / (2.0 * ck * span[i] * delta[i])
                x[i] = x[i] - ak * span[i] * span[i] * g
            x = _clamp(x, bounds)
            k += 1
    except BudgetExhausted:
        pass


def fdgd(ev, x0, bounds, rng, lr=0.15, eps=0.05):
    """Central finite-difference gradient descent — the literal method asked
    for. 2N evals per step (N = dim). Simple, but noise-sensitive; included for
    comparison against SPSA."""
    n = len(x0)
    span = [hi - lo for lo, hi in bounds]
    x = _clamp(list(x0), bounds)
    try:
        while True:
            g = [0.0] * n
            for i in range(n):
                h = eps * span[i]
                xp = list(x); xp[i] = min(bounds[i][1], x[i] + h)
                xm = list(x); xm[i] = max(bounds[i][0], x[i] - h)
                g[i] = (ev(xp) - ev(xm)) / (xp[i] - xm[i] + 1e-12)
            x = _clamp([x[i] - lr * span[i] * span[i] * g[i] / (span[i] + 1e-9)
                        for i in range(n)], bounds)
    except BudgetExhausted:
        pass


def coordinate(ev, x0, bounds, rng, step=0.25, shrink=0.5, min_step=0.02):
    """Pattern / coordinate descent: probe +/- along each axis, accept the best
    move, shrink the step when a full sweep yields no improvement."""
    n = len(x0)
    span = [hi - lo for lo, hi in bounds]
    x = _clamp(list(x0), bounds)
    fx = ev(x)
    s = step
    try:
        while s >= min_step:
            improved = False
            for i in range(n):
                for sign in (+1.0, -1.0):
                    cand = list(x)
                    cand[i] = min(bounds[i][1], max(bounds[i][0], x[i] + sign * s * span[i]))
                    fc = ev(cand)
                    if fc < fx:
                        x, fx, improved = cand, fc, True
            if not improved:
                s *= shrink
    except BudgetExhausted:
        pass


def nelder_mead(ev, x0, bounds, rng, init=0.15):
    """Downhill simplex (Nelder-Mead). Reflect/expand/contract/shrink. Good on
    smooth-ish low-dim problems; restarts a fresh simplex if it collapses."""
    n = len(x0)
    span = [hi - lo for lo, hi in bounds]

    def make_simplex(center):
        S = [_clamp(list(center), bounds)]
        for i in range(n):
            p = list(center); p[i] = min(bounds[i][1], p[i] + init * span[i])
            S.append(_clamp(p, bounds))
        return S

    try:
        simplex = make_simplex(x0)
        fs = [ev(p) for p in simplex]
        while True:
            order = sorted(range(n + 1), key=lambda j: fs[j])
            simplex = [simplex[j] for j in order]
            fs = [fs[j] for j in order]
            centroid = [sum(simplex[j][i] for j in range(n)) / n for i in range(n)]
            worst = simplex[-1]
            refl = _clamp([centroid[i] + 1.0 * (centroid[i] - worst[i]) for i in range(n)], bounds)
            fr = ev(refl)
            if fr < fs[0]:
                exp = _clamp([centroid[i] + 2.0 * (centroid[i] - worst[i]) for i in range(n)], bounds)
                fe = ev(exp)
                simplex[-1], fs[-1] = (exp, fe) if fe < fr else (refl, fr)
            elif fr < fs[-2]:
                simplex[-1], fs[-1] = refl, fr
            else:
                con = _clamp([centroid[i] + 0.5 * (worst[i] - centroid[i]) for i in range(n)], bounds)
                fc = ev(con)
                if fc < fs[-1]:
                    simplex[-1], fs[-1] = con, fc
                else:
                    best = simplex[0]
                    for j in range(1, n + 1):
                        simplex[j] = _clamp([best[i] + 0.5 * (simplex[j][i] - best[i]) for i in range(n)], bounds)
                        fs[j] = ev(simplex[j])
            # restart if the simplex has nearly collapsed
            spread = max(abs(simplex[0][i] - simplex[-1][i]) / (span[i] + 1e-9) for i in range(n))
            if spread < 1e-3:
                simplex = make_simplex(simplex[0])
                fs = [ev(p) for p in simplex]
    except BudgetExhausted:
        pass


def hybrid(ev, x0, bounds, rng, explore_frac=0.35):
    """Coarse random exploration to find a good basin, then SPSA local refine
    from the best point found. A simple but effective global+local mix."""
    explore = max(2, int(ev.budget * explore_frac))
    try:
        ev(_clamp(x0, bounds))
        for _ in range(explore - 1):
            ev([rng.uniform(lo, hi) for lo, hi in bounds])
    except BudgetExhausted:
        return
    spsa(ev, ev.best_x or x0, bounds, rng)


def portfolio(ev, x0, bounds, rng):
    """Split the remaining budget across SPSA, Nelder-Mead and coordinate
    descent (all seeded from x0); the shared Evaluator keeps the global best."""
    methods = [spsa, nelder_mead, coordinate]
    per = max(2, ev.budget // len(methods))
    for m in methods:
        if ev.left <= 0:
            break
        sub = Evaluator(ev.fn, min(per, ev.left))
        sub.n = 0
        # chain: sub delegates to the same underlying fn but caps its slice
        def capped(x, _sub=sub, _ev=ev):
            v = _ev(x)            # counts against the global budget + tracks best
            _sub.n += 1
            if _sub.n >= _sub.budget:
                raise BudgetExhausted()
            return v
        try:
            m(_ShimEval(capped, ev), x0, bounds, rng)
        except BudgetExhausted:
            continue


class _ShimEval:
    """Adapter so meta-optimizers can hand a budgeted sub-evaluator to the
    plain optimizers while still updating the global Evaluator's best/history."""
    def __init__(self, call, real):
        self._call = call
        self._real = real

    def __call__(self, x):
        return self._call(x)

    @property
    def budget(self):
        return self._real.budget

    @property
    def best_x(self):
        return self._real.best_x

    @property
    def left(self):
        return self._real.left


REGISTRY = {
    "random": random_search,
    "spsa": spsa,
    "fdgd": fdgd,
    "coordinate": coordinate,
    "nelder-mead": nelder_mead,
    "hybrid": hybrid,
    "portfolio": portfolio,
}

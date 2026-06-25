// wind_model_test.cpp — WindModel statistics + wind-relative drag (no Qt/daemon).
//   g++ -std=c++17 -I sim/vsim/include sim/vsim/tests/wind_model_test.cpp \
//       sim/vsim/src/physics_core.cpp -o /tmp/wmt && /tmp/wmt
//
// Covers docs/sim-fidelity/01-wind-turbulence.md's test list:
//   - steady-only      -> constant v_wind
//   - gust             -> sinusoid of the right amplitude/period
//   - turbulence       -> RMS ~= turb_sigma (the sqrt(1-a^2) normalisation),
//                         mean ~= 0, and reproducible from a fixed seed
//   - bypass (disabled)-> exactly zero, RNG untouched
//   - physics          -> a hovering craft in steady wind drifts to ground
//                         speed == wind speed (v_rel -> 0): "weathervane drift"
#include "physics_core.h"
#include "wind_model.h"

#include <cmath>
#include <cstdio>

using namespace vsim;

static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg)                                                       \
    do {                                                                       \
        ++g_checks;                                                            \
        if (cond) printf("  ok   %s\n", (msg));                                \
        else { ++g_fails; printf("  FAIL %s (%s:%d)\n", (msg), __FILE__, __LINE__); } \
    } while (0)

int main() {
    const float dt = 1.0f / 1000.0f;  // 1 kHz physics tick

    // --- steady-only: constant wind, no gust/turb -------------------------
    {
        WindModel w;
        WindConfig c;
        c.steady = Vec3(3.0f, 1.5f, 0.0f);
        c.enable = true;
        w.setConfig(c);
        w.seed(42);
        bool constant = true;
        for (int i = 0; i < 2000; ++i) {
            Vec3 v = w.step(dt);
            if (std::fabs(v.x() - 3.0f) > 1e-5f || std::fabs(v.y() - 1.5f) > 1e-5f ||
                std::fabs(v.z()) > 1e-5f)
                constant = false;
        }
        CHECK(constant, "steady-only wind is constant == v_steady");
    }

    // --- gust: sinusoid along the steady direction, right amplitude -------
    {
        WindModel w;
        WindConfig c;
        c.steady      = Vec3(2.0f, 0.0f, 0.0f);  // gust direction = +N
        c.gust_amp    = 1.5f;
        c.gust_period = 4.0f;
        c.enable      = true;
        w.setConfig(c);
        w.seed(1);
        float maxN = -1e9f, minN = 1e9f;
        // Quarter period (1 s) should hit the +peak: 2 + 1.5.
        Vec3 atQuarter(0, 0, 0);
        for (int i = 1; i <= 4000; ++i) {
            Vec3 v = w.step(dt);
            if (i == 1000) atQuarter = v;     // t = 1.0 s = T/4 -> sin = +1
            maxN = std::fmax(maxN, v.x());
            minN = std::fmin(minN, v.x());
        }
        CHECK(std::fabs(atQuarter.x() - (2.0f + 1.5f)) < 0.02f,
              "gust peaks at v_steady + gust_amp at T/4");
        CHECK(std::fabs(maxN - 3.5f) < 0.05f && std::fabs(minN - 0.5f) < 0.05f,
              "gust oscillates steady +/- gust_amp");
    }

    // --- turbulence: RMS ~= sigma, mean ~= 0 ------------------------------
    {
        WindModel w;
        WindConfig c;
        c.turb_sigma = 0.5f;
        c.turb_tau   = 1.0f;
        c.enable     = true;
        w.setConfig(c);
        w.seed(7);
        // Burn in past the filter's transient, then accumulate stats on N.
        for (int i = 0; i < 5000; ++i) w.step(dt);
        double sum = 0.0, sumsq = 0.0;
        const int N = 200000;
        for (int i = 0; i < N; ++i) {
            float x = w.step(dt).x();
            sum += x;
            sumsq += double(x) * x;
        }
        double mean = sum / N;
        double rms  = std::sqrt(sumsq / N);
        // AR(1) output is correlated (length ~ tau/dt ~ 1000 samples), so the
        // sample-mean standard error is ~sigma/sqrt(N*dt/tau) ~ 0.035 here, not
        // the white-noise sigma/sqrt(N). 0.08 keeps the check meaningful (mean
        // is ~6x below the RMS) without being seed-fragile.
        CHECK(std::fabs(mean) < 0.08, "turbulence mean ~= 0");
        CHECK(std::fabs(rms - 0.5) < 0.05, "turbulence RMS ~= turb_sigma");
    }

    // --- reproducibility: same seed -> identical trajectory ---------------
    {
        WindConfig c;
        c.turb_sigma = 0.3f;
        c.enable     = true;
        WindModel a, b;
        a.setConfig(c); b.setConfig(c);
        a.seed(123); b.seed(123);
        bool identical = true;
        for (int i = 0; i < 3000; ++i)
            if (std::fabs(a.step(dt).x() - b.step(dt).x()) > 0.0f) identical = false;
        CHECK(identical, "same seed reproduces the wind trajectory bit-for-bit");
    }

    // --- disabled: exact zero (fast bypass) -------------------------------
    {
        WindModel w;
        WindConfig c;
        c.steady = Vec3(5.0f, 5.0f, 5.0f);
        c.turb_sigma = 1.0f;
        c.enable = false;   // off
        w.setConfig(c);
        w.seed(1);
        bool zero = true;
        for (int i = 0; i < 1000; ++i) {
            Vec3 v = w.step(dt);
            if (v.x() != 0.0f || v.y() != 0.0f || v.z() != 0.0f) zero = false;
        }
        CHECK(zero, "disabled wind is exactly zero (no drag, no RNG draw)");
    }

    // --- physics: weathervane drift -- hover in steady wind, ground speed
    //     converges to the wind speed (air-relative velocity -> 0) ----------
    {
        PhysicsCore phys;
        DroneParams p;            // defaults: mass, linear_drag, gravity
        p.gravity = 9.81f;
        p.linear_drag = 0.2f;     // some drag so wind actually couples
        phys.setParams(p);
        RigidBodyState s0;        // start at rest, level, 10 m up (NED z<0) to
        s0.pos_w = Vec3(0.0f, 0.0f, -10.0f);  // stay clear of the ground clamp
        phys.reset(s0);

        const Vec3 wind(4.0f, 0.0f, 0.0f);   // 4 m/s north
        phys.setWind(wind);
        // Gravity-cancelling body-frame thrust (level craft: -Z body == up).
        const Vec3 hover_F(0.0f, 0.0f, -p.gravity * p.mass);
        const Vec3 zeroT(0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 60000; ++i)      // 60 s @ 1 kHz
            phys.step(hover_F, zeroT, dt);

        float vN = phys.state().vel_w.x();
        CHECK(std::fabs(vN - 4.0f) < 0.05f,
              "hovering craft drifts to ground speed == wind speed (v_rel -> 0)");
    }

    printf("\n%d checks, %d failures\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}

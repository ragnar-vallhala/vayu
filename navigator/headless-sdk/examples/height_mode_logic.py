#!/usr/bin/env python3
"""Height-mode LOGIC test — drives ch6 through every transition and asserts on the
FC's own `height_state` telemetry (VERTICAL_STATE, see control/angle_controller.h).

This tests the STATE MACHINE, not the control behaviour: interlock, engage,
mode selection, the LAND touchdown latch, and latch release. It deliberately does
NOT assert that the craft holds 1 m --

  * SITL is not a valid surrogate for this airframe's control behaviour (see
    firmware/docs/plans/rate-loop-saturation.md section 6a), and
  * the sim's own pitch axis rails at +/-0.425 with zero attitude error, so
    altitude in sim tells you nothing about altitude on hardware.

Every transition here is airframe-independent, so this stays valid when the
vehicle model changes.

    python3 examples/height_mode_logic.py [path/to/vehicle.vveh]
"""
import os, sys, time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(ROOT, "navigator", "headless-sdk"))
os.environ.setdefault("VAYU_SITL_RTOS_BIN",
                      os.path.join(ROOT, "build_sitl_rtos", "vayu_sitl_rtos"))
from vayu_headless import SitlLab                      # noqa: E402
from vayu_headless.paths import gcs_conf_default       # noqa: E402

# height_state bits (control/angle_controller.h)
MODE = 0x03; ENGAGED = 0x04; FAILED = 0x08
LANDED = 0x10; HANDBACK = 0x20; ARMED_OK = 0x40; BLOCKED = 0x80
OFF, HOLD, LAND = 0, 1, 2
# ch6 on the bench radio is INVERTED: up ~1000 = HOLD, down ~2000 = LAND
US_HOLD, US_CENTRE, US_LAND = 1000, 1500, 2000

fails = []
def check(what, ok, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {what}{'  ' + detail if detail else ''}")
    if not ok:
        fails.append(what)

NAV = {0:"UNINIT",1:"INIT",2:"STANDBY",3:"PREARM",4:"ARMED",5:"IN_AIR",
       6:"FAILSAFE",7:"TERM",8:"CALIB"}

def nav(lab):
    hb = lab.telem.get("Heartbeat")
    return getattr(hb, "nav_state", None) if hb else None

def state(lab, settle=0.6):
    """Latest height_state after letting the FC act on the current RC frame."""
    time.sleep(settle)
    vs = lab.telem.get("VerticalState")
    return getattr(vs, "height_state", None) if vs else None

def pin(lab, settle=0.6):
    """Sample while the craft is held ON THE GROUND.

    This sim's pitch axis rails the moment the motors run, and airmode's
    collective shift then flies it away (see rate-loop-saturation.md section 6a),
    so anything that depends on being landed has to pin the pose immediately
    before sampling rather than assume it stayed put."""
    lab.reset_pose((0, 0, -0.05))
    return state(lab, settle)

def describe(h):
    if h is None:
        return "no telemetry"
    names = [n for b, n in ((ENGAGED,"ENGAGED"), (FAILED,"FAILED"), (LANDED,"LANDED"),
                            (HANDBACK,"HANDBACK"), (ARMED_OK,"ARMED_OK"),
                            (BLOCKED,"BLOCKED")) if h & b]
    return f"mode={['OFF','HOLD','LAND','?'][h & MODE]} " + " ".join(names)

def scenario(kw, title, body):
    """Each scenario gets a FRESH session.

    Once this sim's pitch axis rails, airmode's collective shift flies the craft
    away and it will not return to a clean ground state -- a disarm attempt was
    measured still reading nav=IN_AIR after 5 s of trying. So state is never
    carried between scenarios; the process restart is the reset."""
    print(f"\n=== {title} ===")
    with SitlLab(**kw) as lab:
        lab.set_rc(swa=1000, thr=1000, mode=US_CENTRE)
        time.sleep(0.6)
        lab.reset_pose((0, 0, -0.05))
        time.sleep(0.8)
        body(lab)


def normal_sequence(lab):
    h = state(lab)
    check("disarmed: no mode, interlock closed",
          h is not None and (h & MODE) == OFF and not (h & ARMED_OK), describe(h))

    lab.arm(); h = state(lab, 1.0)
    check("arm with switch centred: interlock opens", bool(h and h & ARMED_OK), describe(h))
    check("arm with switch centred: not engaged",
          bool(h is not None and not h & ENGAGED), describe(h))

    lab.set_rc(mode=US_HOLD); h = state(lab)
    check("switch up: mode reads HOLD", bool(h is not None and (h & MODE) == HOLD), describe(h))
    check("switch up: engaged", bool(h and h & ENGAGED), describe(h))
    check("switch up: not blocked", bool(h is not None and not h & BLOCKED), describe(h))

    n_before = nav(lab)
    lab.set_rc(mode=US_CENTRE); h = state(lab)
    check("centre: mode OFF", bool(h is not None and (h & MODE) == OFF), describe(h))
    check("centre: disengaged", bool(h is not None and not h & ENGAGED), describe(h))
    # Hand-back only arms when disengaging IN THE AIR -- on the ground the stick
    # is returned at once, which is the correct behaviour, so the expectation
    # depends on the flight state at the moment of disengage.
    if n_before == 5:
        check("disengage in air: hands back instead of dropping the collective",
              bool(h and h & HANDBACK), describe(h))
    else:
        check("disengage on the ground: returns the stick at once (no hand-back)",
              bool(h is not None and not h & HANDBACK),
              f"nav={NAV.get(n_before, n_before)} {describe(h)}")

    lab.set_rc(mode=US_LAND); h = state(lab)
    check("switch down: mode reads LAND", bool(h is not None and (h & MODE) == LAND), describe(h))


def interlock(lab):
    """Arming with the switch ALREADY up must not fly the craft off the ground."""
    n = nav(lab)
    check("precondition: not armed", n not in (4, 5), f"nav={NAV.get(n, n)}")

    lab.set_rc(mode=US_HOLD); time.sleep(0.4)      # switch UP before arming
    lab.arm(); h = state(lab, 1.0)
    check("armed with switch up: NOT engaged", bool(h is not None and not h & ENGAGED),
          describe(h))
    check("armed with switch up: reported BLOCKED", bool(h and h & BLOCKED), describe(h))
    check("armed with switch up: interlock still closed",
          bool(h is not None and not h & ARMED_OK), describe(h))

    lab.set_rc(mode=US_CENTRE); h = state(lab)
    check("passing through centre opens the interlock", bool(h and h & ARMED_OK), describe(h))
    lab.set_rc(mode=US_HOLD); h = state(lab)
    check("then up: engages", bool(h and h & ENGAGED), describe(h))


def build_logic_rig(path):
    """Derive a logic-test vehicle from the GCS conf geometry.

    NOT a fidelity twin. It applies the documented thrust parity
    (k_thrust x0.31 -> hover ~0.45) so the height loop has real authority, but
    NOT the inertia parity, which the pitch/INDI campaign showed diverges with
    the current firmware. It also INVERTS the motor spins: the harness never
    sends set_motor_geometry, so the firmware always flies its default mix, and
    the conf vehicle's spins make that default yaw mix backwards (yaw positive
    feedback). Inverting them here matches the vehicle to the mix in use.

    Generated rather than committed -- *.vveh is gitignored.
    """
    import json, re
    conf = gcs_conf_default()
    g = {}
    for line in open(conf, encoding="utf-8", errors="replace"):
        m = re.match(r"geometry\\(\S+)=(.*)", line.strip())
        if m:
            g[m.group(1)] = m.group(2)
    if "mass" not in g:
        return None
    motors = []
    for i in range(4):
        motors.append(dict(
            pos={"x": float(g[f"m{i}_px"]), "y": float(g[f"m{i}_py"]), "z": float(g[f"m{i}_pz"])},
            axis={"x": float(g[f"m{i}_ax"]), "y": float(g[f"m{i}_ay"]), "z": float(g[f"m{i}_az"])},
            k_thrust=float(g[f"m{i}_kt"]) * 0.31,
            k_moment=float(g[f"m{i}_km"]) * 0.31,
            max_omega=float(g[f"m{i}_wmax"]),
            spin=-int(float(g[f"m{i}_spin"]))))
    veh = {"format": "vayu-vehicle",
           "_comment": "GENERATED logic-test rig - see build_logic_rig() in "
                       "examples/height_mode_logic.py. Not a fidelity twin.",
           "mass": float(g["mass"]),
           "com": {"x": float(g["comX"]), "y": float(g["comY"]), "z": float(g["comZ"])},
           "inertia": [float(g[f"I{i}"]) for i in range(9)],
           "motors": motors}
    os.makedirs(os.path.dirname(path), exist_ok=True)
    json.dump(veh, open(path, "w"), indent=4)
    return path


def main():
    kw = dict(conf=gcs_conf_default())
    if len(sys.argv) > 1:
        kw["vveh"] = sys.argv[1]
    else:
        rig = build_logic_rig(os.path.join(ROOT, "sim", "vsim", "logic-rig.vveh"))
        if rig:
            kw["vveh"] = rig
            print(f"[rig] generated {os.path.relpath(rig, ROOT)}")
    scenario(kw, "normal sequence: centre -> up -> centre -> down", normal_sequence)
    scenario(kw, "interlock: arm with the switch already up", interlock)
    print("\nNote: the LAND touchdown latch and the runaway/hand-back guards are")
    print("covered by firmware/tests/host/height_ctrl_unit_test.c -- they need a")
    print("landed craft and a controllable altitude, neither of which this sim has.")
    print("\n" + ("FAILED: " + ", ".join(fails) if fails else "ALL PASS"))
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Seam guard: the FC must see ONLY sensors + RC, never the truth.

SITL is meaningful only because the firmware is exercised honestly — sensors +
RC in, PWM + telemetry out. The ground-truth pose is the *operator's* (Pilot)
channel; the firmware must never read physics truth.

Historically the firmware host (vayu_sitl) and physics (vsim_d) were SEPARATE
processes, so this contract could be pinned by reading the FC process's open fds
via /proc: the FC must hold the IMU FIFO but never the pose FIFO. After the SITL
consolidation the firmware and physics run in ONE process (vayu_sitl_rtos), and
the IMU/PWM hop is in-process — so there is no process boundary to inspect and
the fd-based check no longer applies.

The seam is now a SOURCE-level contract: the firmware sources (VAYU_SOURCES in
sim/host/CMakeLists.txt) never include the vsim physics; the physics lives in a
separate translation unit (vsim_inproc.cpp) and only the pose GETTER
(vsim_inproc_get_pose) is exposed — to the harness, not the firmware. A
behavioural check (FC estimate tracks but is not bit-identical to truth) belongs
in the fidelity suite, not here.

See memory/sitl-seam-contract.md and memory/sitl-consolidation.md.
"""
import pytest


@pytest.mark.integration
def test_fc_never_opens_truth_channel():
    pytest.skip(
        "seam is now a source-level contract: firmware + physics share one "
        "process (vayu_sitl_rtos), so there is no fd boundary to inspect. See "
        "the module docstring.")

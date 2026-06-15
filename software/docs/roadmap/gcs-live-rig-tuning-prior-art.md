# Prior-art mechanism reference: automated multirotor control tuning

Companion to [gcs-live-rig-tuning.md](gcs-live-rig-tuning.md). A builder's
teardown of every comparable system found in the
[deep-research survey](gcs-live-rig-tuning.md#feasibility-verdict): how each one
is **set up**, the **algorithm**, the **safety system**, how it checks
**correctness**, its **communication**, and the **control mechanics** (where and
how it excites the plant). Written so an engineer can lift the right pieces into
their own tuner.

> This file is the **at-a-glance** view. For the **from-core** treatment of each
> system — control-theory background, block diagrams, algorithm flowcharts,
> protocol sequences and signal sketches — see
> [gcs-live-rig-tuning-prior-art-deep.md](gcs-live-rig-tuning-prior-art-deep.md).

> **Confidence / currency.** Architecture-level facts (where the optimizer runs,
> excitation type, in-flight vs offline, openness) are from primary docs/papers
> and were adversarially verified (see the study). Lower-level algorithm details
> (model order, RLS, exact cost terms) are established but **version-specific** —
> PX4/ArduPilot track their `main` branch; verify against the current source
> before copying constants. Items marked *(general)* are well-known mechanism, not
> separately source-verified here.

## At a glance

| System | Optimizer runs | Excitation | Where | Open? |
| ------ | -------------- | ---------- | ----- | ----- |
| ArduCopter AUTOTUNE | **onboard FC** (iterative twitch search) | rate step "twitch", axis-by-axis | **in free flight** | GPLv3 |
| ArduCopter SystemID | **offline** (model fit + PID opt) | frequency chirp on a mixer axis | in free flight (log), fit offline | GPLv3 |
| PX4 `mc_autotune` | **onboard FC** (sys-ID + gain calc) | per-axis rate/torque disturbance (doublet) | in free flight (alt-stabilized) | BSD-3 |
| PX4 manual/step guide | **human** | stick step input | in free flight | BSD-3 (docs) |
| CIFER + CONDUIT/Simulink (academic) | **offline desktop** | frequency sweeps (pilot/autopilot) | in free flight (log), fit offline | proprietary tools, published method |
| MathWorks Closed-Loop PID Autotuner | **in the block** (online FRF→gains) | sinusoidal perturbation at PID **output** | sim, deployable to HW; free flight | proprietary |
| Safe Bayesian Opt (Berkenkamp et al.) | **offline/host optimizer** picks candidates, evaluates on HW | controller run per candidate (closed-loop trial) | in free flight | published method |
| Thrust stands (RCbenchmark/Tyto) | n/a (characterization) | motor/prop sweep on a stand | **on a bench** (component, not closed-loop PID) | proprietary HW + scripting |

**None** runs a **live GCS optimizer with the FC as a per-pass cost-reporting
executor**, and **none** tunes on a **mechanically-constrained rig** — those two
are ours (see the study's novelty section).

---

## 1. ArduCopter AUTOTUNE (onboard, in-flight, twitch search)

Source: <https://ardupilot.org/copter/docs/autotune.html>

- **Setup.** Tune one axis-set per flight; needs calm conditions and open space
  (the craft repeatedly twitches and drifts). Operator flies to a safe altitude
  in AltHold/Loiter, flips an RC aux switch (or mode) to AUTOTUNE, and **holds
  position while keeping a hand on the sticks**. An aggressiveness parameter
  (`AUTOTUNE_AGGR`, ~0.05–0.10) trades responsiveness vs overshoot; `AUTOTUNE_AXES`
  selects roll/pitch/yaw.
- **Control mechanics.** Injects a **rate "twitch"** (a brief commanded angular-
  rate step) directly into the rate controller of the axis under test, then
  watches the gyro response. *(general)* Excitation is generated **onboard**.
- **Algorithm.** Per-axis iterative search, not a model fit: ramp **rate-D** up
  until the response just starts to oscillate/bounce-back, back off; ramp
  **rate-P** up to a target response; set **angle-P** from the achieved bandwidth;
  apply a safety margin scaled by `AUTOTUNE_AGGR`. Converges axis-by-axis over
  many twitches. *(general)*
- **Safety.** Original gains are kept; **moving the sticks hands control back** to
  the pilot instantly; disarming or leaving the mode **restores the pre-tune
  gains** unless the operator explicitly saves them (land + low-throttle save, or
  switch). Self-aborts on excessive attitude error. *(general)*
- **Correctness.** Convergence is per-axis to a target rate-response shape; the
  proof is the **post-tune test flight** (the pilot flies and decides to save).
  No model/coherence metric.
- **Communication.** Self-contained on the FC; the GCS (Mission Planner/QGC) only
  sets params (MAVLink `PARAM_SET`) and arms the mode. Results are the updated PID
  params, saved to the FC.
- **Takeaway for us.** The *twitch + bounded per-axis search* is the spiritual
  ancestor of our doublet rollout — but the search lives on the FC. We move that
  search to the GCS. Their **stick-override abort** maps onto our operator gesture.

## 2. ArduCopter SystemID (chirp excitation + offline model/PID fit)

Sources: <https://ardupilot.org/copter/docs/systemid-model-development.html>,
<https://ardupilot.org/copter/docs/common-systemid-mode-operation.html>

- **Setup.** A dedicated `SystemID` flight mode. The operator hovers and triggers
  a sweep; logs are pulled afterward for desktop analysis. Params pick the axis
  and magnitude.
- **Control mechanics.** Injects a **frequency-sweep (chirp)** by *superimposing
  the waveform on the mixer inputs* via `SID_AXIS` (10 = roll, 11 = pitch,
  12 = yaw) — "direct input to the regarded system." The chirp is generated
  onboard (`chirp_input.init()/update()`, `actuator_roll_sysid()` in
  `mode_systemid.cpp`).
- **Algorithm.** Two clean stages: (1) **offline** frequency-domain fit — plant
  transfer-function parameters are optimized to best match the collected flight-
  data frequency responses; (2) the identified model is then used to **optimize
  the PID** in a separate step. Identification is decoupled from gain tuning.
- **Safety.** Short bounded sweeps in a stable hover; pilot retains override.
- **Correctness.** Goodness-of-fit of the transfer function to the measured FRF;
  then verify the optimized PID in flight.
- **Communication.** Onboard logging (dataflash) → offline tools on a PC; gains
  uploaded back as params.
- **Takeaway for us.** Confirms **FC-side chirp at the mixer/setpoint level** is a
  proven excitation injection point — exactly our "override the axis setpoint."
  The clean **ID-then-tune** split is an alternative to our direct cost search; we
  chose direct cost (no model), but their injection mechanics transfer directly.

## 3. PX4 `mc_autotune` (fully onboard sys-ID + gain calc, in-flight)

Sources: <https://docs.px4.io/main/en/config/autotune_mc>,
<https://docs.px4.io/main/en/msg_docs/AutotuneAttitudeControlStatus>

- **Setup.** Vehicle must be **flying in an altitude-stabilized mode** (not a
  bench). Operator triggers autotune from QGroundControl; it runs hands-off.
- **Control mechanics.** The flight stack **applies a small disturbance to each
  axis** (a normalized torque/rate setpoint perturbation; effectively a doublet),
  recording the filtered input `u_filt` (normalized torque setpoint) and output
  `y_filt` (angular velocity).
- **Algorithm.** **Onboard system identification**: fit a discrete-time SISO model
  (a low-order 2-pole/2-zero ARX, via recursive least squares *(general/version-
  specific)*) — the `AutotuneAttitudeControlStatus` uORB message carries `coeff`
  (model coefficients), `coeff_var` (variance), and a **fitness**. From the
  identified model it computes the gains directly: `kc, ki, kd, kff, att_p`.
  Processes axes **sequentially roll → pitch → yaw**.
- **Safety.** **Reverts unstable gains in real time onboard**; requires the
  altitude-stabilized envelope; operator can abort by switching mode. The
  disturbance amplitude is deliberately small.
- **Correctness.** The onboard **fitness** + coefficient **variance** gate
  acceptance; final check is a verification flight.
- **Communication.** Triggered over **MAVLink** from QGC; status streamed via the
  uORB→MAVLink bridge; resulting params written to the FC.
- **Takeaway for us.** The strongest precedent for **FC-side excitation + onboard
  scoring** — PX4 already computes a fitness on the FC, which is our "FC computes
  cost." The difference: PX4 *also* computes the gains onboard from a model; we
  keep the **iterative gain choice on the GCS** and skip the model (direct cost).

## 4. PX4 manual / step-input tuning guide (human-in-the-loop baseline)

Source: <https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter>

- **Setup / mechanics / algorithm.** No optimizer. Hover, **give a fast step
  input via the stick** ("push roll to one side, then let it snap back"), watch
  for oscillation/overshoot, **land before changing a parameter**, repeat.
- **Safety.** The land-between-changes rule is the safety system; slow throttle
  ramps to check for oscillation first.
- **Takeaway.** This is the manual loop our autotuner automates; the **step
  doublet** is the same excitation, and "land between changes" is what our
  **operator-gated, re-stabilize-between-rollouts** protocol replaces with a rig.

## 5. CIFER + CONDUIT / Simulink (academic offline frequency-domain)

Sources: <https://www.academia.edu/54509862/Frequency_Response_System_Identification_and_Flight_Controller_Tuning_for_Quadcopter_UAV>,
<https://arxiv.org/pdf/1508.04886>, <https://www.sjsu.edu/researchfoundation/docs/AHS_2015_Wei.pdf>,
<https://arxiv.org/pdf/0804.3881>

- **Setup.** Instrument the craft, fly **frequency sweeps in hover**, axis by
  axis. Sweeps are input by a **human pilot via RC**, or (better) **generated by
  the autopilot** — a human "cannot accurately cover the whole frequency needed,"
  so an onboard-generated sweep yields higher-quality data (Adiprawita et al.).
- **Control mechanics.** Frequency-sweep excitation per axis, logged.
- **Algorithm.** **CIFER** (Comprehensive Identification from FREquency Response)
  does multi-window FFT → frequency response → fits SISO transfer functions in the
  **frequency domain** (offline). Then PID gains are **optimized offline** against
  handling-qualities specs with **CONDUIT** or **MATLAB Simulink Design
  Optimization**. Gains are uploaded to the Pixhawk (overwritten via QGC).
- **Safety.** Conventional piloted flight with a safety pilot; the heavy lifting
  is offline, so nothing risky happens autonomously on the vehicle.
- **Correctness.** The **coherence function** validates each frequency point of
  the FRF (data quality), and the model fit cost + the CONDUIT spec margins gauge
  the result; verified by flight test.
- **Communication.** Onboard log → desktop tools → params back via QGC/MAVLink.
  Pure **upload-conduit** GCS role.
- **Takeaway for us.** The rigorous end of the field. **Coherence** is a correctness
  idea worth stealing — a per-window data-quality gate before trusting a cost. But
  it's offline and model-based; we trade model fidelity for closed-loop directness.

## 6. MathWorks Closed-Loop PID Autotuner

Sources: <https://www.mathworks.com/help/slcontrol/ug/pid-controller-tuning-for-a-uav-quadcopter.html>,
<https://www.mathworks.com/help/slcontrol/ug/closedlooppidautotuner.html>,
<https://www.mathworks.com/help/uav/ug/pid-autotuning-for-uav-quadcopter.html>

- **Setup.** A Simulink block in the control loop; primarily demonstrated in
  simulation, but **deployable to real hardware** via generated C/C++ ("use the
  Closed-Loop PID Autotuner on hardware to perform the same process").
- **Control mechanics.** Injects **sinusoidal perturbation signals at the OUTPUT
  of each existing PID controller** (8 loops in the quad example) — i.e. it
  perturbs the actuator command, not the setpoint.
- **Algorithm.** Estimates the plant **frequency response in closed loop, in real
  time**, from the perturbation experiment, then computes PID gains from the FRF.
  Self-contained (the optimizer is in the block, not external).
- **Safety.** Runs the experiment around the operating point with bounded
  perturbations; closed-loop so the existing controller keeps the craft stable
  during the experiment.
- **Correctness.** FRF-based; the block reports estimated margins.
- **Takeaway for us.** The "**perturb at the controller output, identify in closed
  loop**" trick is an alternative excitation injection point (actuator vs
  setpoint) and avoids needing clean setpoint authority — worth noting if setpoint
  override proves awkward on the FC.

## 7. Safe Bayesian Optimization on hardware (Berkenkamp et al.)

Source: <https://las.inf.ethz.ch/files/berkenkamp16safe.pdf>

- **Why it matters most to us.** This is the **closest architectural cousin**: an
  **external optimizer proposes candidate controller parameters, each is evaluated
  by a real closed-loop trial on the quadrotor, a scalar performance is measured,
  and the optimizer picks the next candidate** — exactly our GCS-as-optimizer /
  evaluate-a-pass loop, minus the rig.
- **Algorithm.** **Safe Bayesian Optimization (SafeOpt)**: a Gaussian-process
  model of performance-vs-parameters with a **safety constraint**, so the search
  only tries parameter sets predicted to stay above a safety threshold — it never
  knowingly evaluates a destabilizing gain. Sample-efficient (few trials).
- **Safety / correctness.** The safety constraint is the headline feature: it
  bounds exploration to a "safe set," directly addressing our "the search probes
  unstable gains by design" risk.
- **Takeaway for us.** If our existing SITL optimizer struggles with real noise
  (the study's deferred "optimizer selection" question), **SafeOpt-style Bayesian
  optimization is the strongest candidate** — sample-efficient (few real
  rollouts) and safety-constrained (won't probe known-bad gains). Strongly
  consider for the live phase even though we keep the rig as the hard safety net.

## 8. Adjacent (not closed-loop PID tuners, but useful inputs)

- **Thrust stands — RCbenchmark / Tyto Robotics**
  (<https://www.tytorobotics.com/pages/rcbenchmark-software>,
  <https://www.tytorobotics.com/blogs/software-troubleshooting/automatic-control-pid>):
  bench rigs that **characterize a single motor+prop** (thrust, torque, current,
  efficiency, and a PID for the *stand's own* control). **Not** closed-loop
  airframe PID tuning. Use them to **feed real motor/prop constants into the SITL
  model**, improving sim-then-hardware transfer — not to tune the FC.
- **Data-driven parameter ID + RL** (Eschmann/Albani/Loianno 2024,
  <https://arxiv.org/pdf/2404.07837>): identifies inertia/thrust/motor-delay from
  ~73 s of **free flight**, then trains an RL policy. Confirms free-flight data
  collection dominates; a pointer for a future model-based or learned controller,
  not classical PID.
- **Betaflight**: no built-in closed-loop PID autotuner *(general, not
  source-verified here)* — relies on presets, filters, and manual tuning. The
  research found no confirmed Betaflight autotune claim.

---

## Synthesis — what to borrow, mapped to our subsystems

| Our subsystem | Best precedent to copy |
| ------------- | ---------------------- |
| FC-side excitation injection point | ArduCopter SystemID (chirp on the mixer/setpoint axis); MathWorks (perturb at PID output) as a fallback |
| Excitation waveform | PX4 doublet + ArduPilot/CIFER chirp — support both (we already do) |
| Onboard cost/score | PX4 onboard **fitness** + coefficient variance |
| Data-quality gate | CIFER **coherence** per window → reject bad rollouts before scoring |
| Optimizer (if SITL one underperforms) | **SafeOpt / Bayesian optimization** (sample-efficient + safety-constrained) |
| Abort / handback | ArduPilot AUTOTUNE **stick-override** → our operator gesture |
| Keep-old-gains-until-saved | ArduPilot AUTOTUNE / PX4 revert → our "never persist intermediates; explicit Apply" |
| Sim-then-hardware | Standard across PX4/ArduPilot; feed real motor constants from a thrust stand |

The two things **no precedent gives us** — a **live GCS optimizer with the FC as a
per-pass executor**, and a **constrained-rig closed-loop rollout** — are where the
design work is genuinely original, and where the study's open risks (rig-induced
dynamics bias, reliable command/result over NavLink v2) have no off-the-shelf
answer to copy.

## Changelog

| Date       | Author          | Description                          |
| ---------- | --------------- | ------------------------------------ |
| 15/06/2026 | ragnar-vallhala | Initial prior-art mechanism reference |

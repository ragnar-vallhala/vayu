# sim_gazebo — Gazebo Harmonic + quadrotor world

Phase 3 of the SITL plan: Gazebo as the physical plant. Phase 4 will wire it to the Renode-emulated firmware via a co-sim bridge (motors PWM out → rotor thrust in; IMU out → I²C BMX160 mock).

## Install Gazebo Harmonic (one-time)

Linux Mint 22 / Ubuntu 24.04 (Noble). The Mint codename `wilma` confuses OSRF's repo — use `noble` directly in the source list:

```bash
sudo apt update && sudo apt install -y curl lsb-release gnupg
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install -y gz-harmonic
```

≈600 MB on disk. After install:

```bash
gz sim --version    # should print 8.x (Harmonic = gz-sim 8)
```

## Run the quadrotor world

From this repo's root:

```bash
gz sim tools/sim_gazebo/worlds/vayu_quad.sdf
```

First launch downloads the X3 UAV model from Fuel (≈10 MB, cached at `~/.gz/fuel`). Then the GUI opens, the quad is on the ground; press the play button (▶) to step physics.

### Headless smoke test (no GUI)

```bash
gz sim -s -r --iterations 1000 tools/sim_gazebo/worlds/vayu_quad.sdf
```

Should exit cleanly after 1 s of sim time.

## Verify motor control works

While the world is running:

```bash
# List topics; expect a /world/vayu_quad_world/model/vayu_quad/... namespace
gz topic -l | grep vayu_quad

# Spin the rotors at idle (model takes Actuators messages on its
# motor_speed topic — exact name depends on the X3 model variant)
gz topic -t /vayu_quad/gazebo/command/motor_speed \
  -m gz.msgs.Actuators \
  -p 'velocity: [700, 700, 700, 700]'
```

## What's the X3 UAV Config 1?

Gazebo's reference quadrotor with sensors (the bare `X3 UAV` variant has *only* rotor links — no sensors at all, found that out the hard way). `Config 1` adds:

- IMU sensor on the base link → publishes at `…/sensor/imu_sensor/imu`
- Magnetometer → `…/sensor/magnetometer/magnetometer`
- Air-pressure → `…/sensor/air_pressure/air_pressure`
- Front-facing camera → `…/sensor/camera_front/image`

Our world layers on top of that: the four `gz::sim::systems::MulticopterMotorModel` plugins (rotor RPM → thrust + drag) and the three sensor system plugins (`Imu`, `Magnetometer`, `AirPressure`) that the stock `quadcopter.sdf` example omits.

Note the rotor naming difference between variants: `X3 UAV` uses link names like `X3/rotor_0`, but `X3 UAV Config 1` drops the `X3/` prefix and uses `rotor_0`. The `MulticopterMotorModel` plugins in our world reference the un-prefixed names.

## Phase 4 hookup (next session)

A Python bridge that, every Gazebo step:
1. Reads `/world/vayu_quad_world/model/vayu_quad/link/base_link/sensor/imu_sensor/imu` → packs into a BMX160 register response → answers Renode's I²C reads (Phase 5 BMX160 mock).
2. Reads vayu's emulated TIM PWM CCR registers → converts duty cycle → rotor velocity → publishes to the four `/.../motor_speed` topics.

The closed loop runs vayu's actual control code against Gazebo's physics — same firmware as on hardware.

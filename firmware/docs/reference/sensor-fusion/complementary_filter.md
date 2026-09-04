# Complementary Filter

The complementary filter is a simple method to estimate orientation by fusing accelerometer and gyroscope data. It is computationally efficient and works well for basic stabilization. It effectively balances the drift of the gyroscope with the stability of the accelerometer.

## Mathematical Principle

The filter operates on the principle of frequency-domain complementarity.

### 1. Sensor Characteristics

- **Gyroscope**: High-frequency accuracy. Integrating angular velocity $(\omega)$ gives a precise short-term attitude but accumulates a low-frequency bias (drift) $\int \omega \, dt$.
- **Accelerometer**: Low-frequency accuracy. In a static or quasi-static state, the accelerometer measures the gravity vector, which can be converted to Roll and Pitch angles $(\theta_{acc})$. However, it is sensitive to vibrations and transient accelerations (high-frequency noise).

### 2. The Filter Equation

The filter combines these two sources using a single coefficient $\alpha$ (the low-pass filter factor):

$$ \theta_{t} = \alpha \cdot (\theta_{t-1} + \omega \cdot dt) + (1 - \alpha) \cdot \theta_{acc} $$

- **High-pass part**: $\alpha \cdot (\theta_{t-1} + \omega \cdot dt)$ integrates the gyro and keeps high-frequency changes.
- **Low-pass part**: $(1 - \alpha) \cdot \theta_{acc}$ uses the accelerometer and keeps the low-frequency stability.

For **Yaw**, the filter uses the magnetometer-derived yaw $(\psi_{mag})$ instead of the accelerometer:
$$ \psi_{t} = \alpha \cdot (\psi_{t-1} + \omega_z \cdot dt) + (1 - \alpha) \cdot \psi_{mag} $$

## Implementation Details

The implementation in `vayu` is found in `src/est/sensor_fusion.c` within the function `m_complementary_filter`.

### Variables & Data Structures

- **State**: Stored in `attitude_t *ori` (members: `roll`, `pitch`, `yaw`).
- **Inputs**:
  - `gx, gy, gz`: Raw gyroscope rates in **degrees per second (dps)**.
  - `ax, ay, az`: Accelerometer readings in **$m/s^2$**.
  - `mx, my, mz`: Magnetometer readings in **$\mu T$**.
- **Time**: `dt` is the sample period in seconds (e.g., `0.001f` for 1kHz).

### Step-by-Step Logic

1.  **Calculate Reference**: `m_acc_mag` is called to compute the current "noisy" orientation (`acc_mag_ori`) from gravity and magnetism.
    - Accelerometer angles are calculated using `atan2`.
    - Yaw is tilt-compensated using the current roll and pitch.
2.  **Apply Filter**: The filter equation is applied to each axis.
3.  **Normalization**: Angles are wrapped to the $[-180, 180]$ range to prevent overflow/discontinuity.

### Units

| Variable          | Unit                            |
| :---------------- | :------------------------------ |
| Roll/Pitch/Yaw    | Degrees ($^\circ$)              |
| Gyro (gx, gy, gz) | Degrees/second ($^\circ/s$)     |
| Acc (ax, ay, az)  | Meters/second squared ($m/s^2$) |
| Mag (mx, my, mz)  | Microtesla ($\mu T$)            |

## Tuning

Configured in `include/variables.h`:

- `SF_COMPLEMENTARY_ALPHA`: Typically set to `0.98f`. A higher value trusts the gyro more (smoother but more drift-prone), while a lower value trusts the accelerometer more (more responsive to gravity but shakier).

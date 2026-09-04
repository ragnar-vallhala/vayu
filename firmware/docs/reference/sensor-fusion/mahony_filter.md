# Mahony AHRS Filter

The Mahony filter is a robust Attitude and Heading Reference System (AHRS) filter that uses quaternions to represent orientation. It provides high-performance attitude estimation while avoiding the singularities associated with Euler angles (gimbal lock) and is more computationally efficient than a full Kalman Filter.

## Mathematical Principle

The Mahony filter utilizes a Proportional-Integral (PI) feedback mechanism to correct the orientation estimate based on the observed gravity and magnetic vectors.

### 1. State Representation

The attitude is represented as a unit quaternion $\mathbf{q} = [w, x, y, z]$.

### 2. Direction Extraction (Body Frame)

The gravity vector $\mathbf{v}$ and magnetic vector $\mathbf{w}$ in the **earth frame** are known. We rotate these into the **body frame** using the current quaternion $\mathbf{q}$:

$$ \mathbf{v}_{est} = \begin{bmatrix} 2(q_1q_3 - q_0q_2) \\ 2(q_0q_1 + q_2q_3) \\ q_0^2 - q_1^2 - q_2^2 + q_3^2 \end{bmatrix} $$

### 3. Error Calculation

The angular error $\mathbf{e}$ is determined by the cross product between the measured sensor vectors and the estimated vectors:

$$ \mathbf{e} = (\mathbf{a}_{meas} \times \mathbf{v}_{est}) + (\mathbf{m}_{meas} \times \mathbf{w}_{est}) $$

This error represents the rotation needed to align the estimated orientation with the physical references (gravity and north).

### 4. PI Feedback Loop

The error is used to adjust the gyroscope rates $(\omega_{meas})$. The integral term $(\Omega_{bias})$ accounts for constant gyroscope bias over time.

$$ \Omega_{bias} = \Omega_{bias} + K_i \cdot \mathbf{e} \cdot dt $$
$$ \omega_{corr} = \omega_{meas} + K_p \cdot \mathbf{e} + \Omega_{bias} $$

### 5. Quaternion Integration

The orientation is updated by integrating the rate of change:

$$ \dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \omega_{corr} $$
$$ q_{t+dt} = q_t + \dot{\mathbf{q}} \cdot dt $$
The resulting quaternion is normalized to maintain unit length.

## Implementation Details

The implementation in `vayu` is found in `src/est/sensor_fusion.c` within the function `m_mahony_filter`.

### Variables & Data Structures

- **State**: Stored in `ori->q` (type `quaternion_t`). Calculated Euler angles are stored in `ori->roll, ori->pitch, ori->yaw`.
- **Inputs**:
  - `gx, gy, gz`: Raw gyroscope rates in **degrees per second (dps)**. (Internal implementation converts these to **radians per second** before integration).
  - `ax, ay, az`: Accelerometer readings in **$m/s^2$**.
  - `mx, my, mz`: Magnetometer readings in **$\mu T$**.
- **Feedback**: `integralFBx, integralFBy, integralFBz` are static variables storing the integrated error.

### Step-by-Step Logic

1.  **Normalization**: Sensor inputs (Acc and Mag) are normalized to unit vectors.
2.  **Reference Rotation**: Earth field is rotated into body frame.
3.  **Error Computation**: Cross products are calculated axes by axes.
4.  **PI Logic**: Error is scaled by $K_p$ and $K_i$ and added to the gyro rates.
5.  **Integration**: Quaternion values are updated and then strictly re-normalized.
6.  **Conversion**: `m_quat_to_euler` converts the internal quaternion to Euler angles for use by the flight controller.

### Units

| Variable                  | Unit                            |
| :------------------------ | :------------------------------ |
| Roll/Pitch/Yaw            | Degrees ($^\circ$)              |
| Internal Gyro Integration | Radians ($rad$)                 |
| Acc (ax, ay, az)          | Meters/second squared ($m/s^2$) |
| Mag (mx, my, mz)          | Microtesla ($\mu T$)            |

## Tuning

Configured in `include/variables.h`:

- `SF_MAHONY_KP`: Proportional gain. Controls how fast the sensors pull the estimate towards the reference. Too high causes oscillation; too low causes sluggish response.
- `SF_MAHONY_KI`: Integral gain. Gradually removes steady-state drift in the gyroscope.

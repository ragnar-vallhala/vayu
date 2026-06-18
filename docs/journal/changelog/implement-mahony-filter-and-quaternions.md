# Implementation of Mahony AHRS Filter and Quaternions

## Summary

Transitioned the attitude estimation system from a basic Euler-based complementary filter to a robust Mahony AHRS filter. This change includes moving to quaternions for all internal computations to ensure stability and avoid gimbal lock.

## Mathematical Details

The filter estimates the orientation by fusing gyroscope integration with accelerometer and magnetometer corrections.

### 1. Error Estimation

The error $\mathbf{e}$ is calculated by comparing the measured gravity and magnetic field vectors with their estimates derived from the current orientation quaternion $\mathbf{q}$.

$$ \mathbf{e} = (\mathbf{a}_{measured} \times \mathbf{v}_{estimated}) + (\mathbf{m}_{measured} \times \mathbf{w}_{estimated}) $$

Where:

- $\mathbf{v}_{estimated}$ is the gravity vector rotated into the body frame using $q$:
  $v_x = 2(q_1q_3 - q_0q_2)$
  $v_y = 2(q_0q_1 + q_2q_3)$
  $v_z = q_0^2 - q_1^2 - q_2^2 + q_3^2$
- $\mathbf{w}_{estimated}$ is the earth's magnetic field rotated into the body frame.

### 2. Feedback Control

The error is used in a PI controller to compensate for gyroscope bias and noise.

$$ \mathbf{\Omega}_{bias} = \int K_i \cdot \mathbf{e} \cdot dt $$
$$ \mathbf{\omega}_{compensated} = \mathbf{\omega}_{measured} + K_p \cdot \mathbf{e} + \mathbf{\Omega}_{bias} $$

### 3. Quaternion Integration

The orientation is updated using the derivative of the quaternion:

$$ \dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \mathbf{\omega}_{compensated} $$
$$ \mathbf{q}_{t+dt} = \mathbf{q}\_t + \dot{\mathbf{q}} \cdot dt $$

The resulting quaternion is then normalized to maintain unit length.

## Chosen Constants

The following parameters were tuned for optimal performance on the drone:

| Parameter                | Value  | Description                                      |
| ------------------------ | ------ | ------------------------------------------------ |
| `SF_MAHONY_KP`           | 1.5f   | Proportional gain for stability and convergence. |
| `SF_MAHONY_KI`           | 0.005f | Integral gain to compensate for gyro drift.      |
| `SF_COMPLEMENTARY_ALPHA` | 0.98f  | Fallback alpha for Euler-based filter.           |

## Root Cause Fixed: Gyro Sensitivity

During implementation, it was discovered that the angular rates were being integrated in degrees per second rather than radians per second. This caused extreme sensitivity (57.3x). This was fixed by converting all gyro inputs to radians before applying them to the filter.

## Impact

- **No Gimbal Lock**: Quaternions permit full 3D rotation without singularities.
- **Improved Accuracy**: The Mahony filter provides better high-frequency stability and faster convergence than the previous complementary filter.
- **Centralized Tunability**: All parameters are now exposed in `include/variables.h`.

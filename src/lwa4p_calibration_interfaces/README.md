# lwa4p_calibration_interfaces

**Author:** Dmitrii Ognev

Service definitions for teaching and replaying hand-eye calibration poses.

## Services

| Service | Request | Response | Used by |
|---|---|---|---|
| `SendToPose` | `float64[] positions` — joint positions [rad] | `bool success`, `string message` | `pose_teacher` in `lwa4p_calibration_poses` |

`save_pose` and `run_calibration_sequence` take no input and use `std_srvs/srv/Trigger`.

## How to compile

```bash
colcon build --packages-select lwa4p_calibration_interfaces
```

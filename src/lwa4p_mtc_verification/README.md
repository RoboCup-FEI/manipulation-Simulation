# LWA4P MTC verification

Plan and execute a small arm/gripper sequence, then verify measured joint motion in Isaac.

| Interface | Behavior |
| --- | --- |
| `/lwa4p/test_ptp` (`std_srvs/srv/Trigger`) | Run the MTC sequence; respond after verification |
| Concurrent request | Return `success: false`, `message: BUSY` |
| `report_path` parameter | Required JSON report filename |

The main thread spins a `SingleThreadedExecutor`.
One worker executes the MTC task; Humble's deferred service response keeps callbacks available.
This is a simulation test utility, not the manipulation skill server.

| Files | Responsibility |
| --- | --- |
| `verification_node.cpp` | Node and executor setup |
| `verification_server_ros.cpp` | Service, admission, worker, and report output |
| `execution_verification.cpp` | MTC integration and measured joint feedback |
| `joint_motion_verification.cpp` | ROS-free checks of final positions and observed motion |

Build and run through the [simulation workflow](../../isaac/moveit/README.md).
The checks expect the simulated two-finger WSG model and are not launched on real hardware.

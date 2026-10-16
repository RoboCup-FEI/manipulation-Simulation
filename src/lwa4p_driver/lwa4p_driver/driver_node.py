import bisect
import math
import os
import signal
import threading
import time

import canopen
import rclpy
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointJog
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from lwa4p_driver import ds402

DEG = math.pi / 180.0


class Trajectory:

    def __init__(self, times, positions, velocities=None):
        self.t = times
        self.q = positions
        self.v = velocities
        self.duration = times[-1]

    def sample(self, t):
        if t <= self.t[0]:
            return list(self.q[0])
        if t >= self.duration:
            return list(self.q[-1])
        k = bisect.bisect_right(self.t, t) - 1
        t0, t1 = self.t[k], self.t[k + 1]
        h = t1 - t0
        s = (t - t0) / h
        out = []
        for j in range(len(self.q[0])):
            p0, p1 = self.q[k][j], self.q[k + 1][j]
            if self.v is None:
                out.append(p0 + s * (p1 - p0))
                continue
            m0, m1 = self.v[k][j] * h, self.v[k + 1][j] * h
            s2, s3 = s * s, s * s * s
            out.append((2 * s3 - 3 * s2 + 1) * p0 + (s3 - 2 * s2 + s) * m0
                       + (-2 * s3 + 3 * s2) * p1 + (s3 - s2) * m1)
        return out


class Lwa4pDriver(Node):
    def __init__(self):
        super().__init__("lwa4p_driver")
        p = self.declare_parameter
        self.channel = p("can_channel", "can0").value
        self.node_ids = list(p("node_ids", [3, 4, 5, 6, 7, 8]).value)
        self.joint_names = list(p("joint_names", [f"arm_{i}_joint" for i in range(1, 7)]).value)
        self.signs = list(p("joint_signs", [1.0] * 6).value)
        self.offsets = [o * DEG for o in p("joint_offsets_deg", [0.0] * 6).value]
        self.rate = p("rate_hz", 20.0).value
        self.lookahead = p("lookahead_s", 0.15).value
        self.max_vel = p("max_velocity_deg_s", 30.0).value * DEG
        self.accel_mdeg = p("acceleration_deg_s2", 60.0).value * 1000.0
        self.limit_margin = p("limit_margin_deg", 3.0).value * DEG
        self.start_tol = p("start_tolerance_deg", 2.0).value * DEG
        self.path_tol = p("path_tolerance_deg", 8.0).value * DEG
        self.goal_tol = p("goal_tolerance_deg", 0.5).value * DEG
        self.goal_time = p("goal_time_tolerance_s", 3.0).value
        self.enable_settle = p("enable_settle_s", 1.0).value
        controller = p("controller_name", "arm_controller").value
        assert len(self.node_ids) == len(self.joint_names) == len(self.signs) == len(self.offsets)

        # ---- CAN ----
        self.can_lock = threading.Lock()
        try:
            with open(f"/sys/class/net/{self.channel}/flags") as f:
                up = int(f.read(), 16) & 0x1
        except OSError:
            up = False
        if not up:
            msg = (f"{self.channel} is missing or DOWN (PCAN unplugged? udev rule not installed?). "
                   f"Fix: sudo ip link set {self.channel} up type can bitrate 500000")
            self.get_logger().fatal(msg)
            raise SystemExit(msg)
        self.net = canopen.Network()
        self.net.connect(interface="socketcan", channel=self.channel)
        self.nodes = []
        for nid in self.node_ids:
            n = canopen.RemoteNode(nid, canopen.ObjectDictionary())
            n.sdo.RESPONSE_TIMEOUT = 0.3
            self.net.add_node(n)
            self.nodes.append(n)
        with self.can_lock:
            self.limits = []
            for i, n in enumerate(self.nodes):
                lo, hi = ds402.sw_limits(n)
                self.limits.append(sorted((self.to_rad(i, lo), self.to_rad(i, hi))))
            self.q = [self.to_rad(i, ds402.position(n)) for i, n in enumerate(self.nodes)]


        self.state_lock = threading.Lock()
        self.enabled = False
        self.fault = ""
        self.traj = None
        self.traj_t0 = 0.0
        self.traj_result = None
        self.trace, self.trace_path = None, ""
        self.q_prev, self.t_prev = list(self.q), time.time()
        self.qd = [0.0] * len(self.nodes)

        # ---- ROS interfaces ----
        cb = ReentrantCallbackGroup()
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_service(Trigger, "~/enable", self.srv_enable, callback_group=cb)
        self.create_service(Trigger, "~/disable", self.srv_disable, callback_group=cb)
        self.create_service(Trigger, "~/reset", self.srv_reset, callback_group=cb)
        self.action = ActionServer(
            self, FollowJointTrajectory, f"{controller}/follow_joint_trajectory",
            execute_callback=self.execute, goal_callback=self.on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=cb)
        self.jog_init(cb)

        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        self.get_logger().info(
            f"LWA4P on {self.channel}, nodes {self.node_ids}. Drives DISABLED - "
            f"call 'ros2 service call /lwa4p_driver/enable std_srvs/srv/Trigger'")

    # ---------- unit conversion ----------
    def to_rad(self, i, mdeg):
        return self.signs[i] * mdeg / 1000.0 * DEG + self.offsets[i]

    def to_mdeg(self, i, rad):
        return (rad - self.offsets[i]) / DEG * 1000.0 / self.signs[i]

    # ---------- control loop ----------
    def loop(self):
        period = 1.0 / self.rate
        cycle = 0
        while self.running and rclpy.ok():
            t_start = time.time()
            try:
                with self.can_lock:
                    q = [self.to_rad(i, ds402.position(n)) for i, n in enumerate(self.nodes)]
                    sw = None
                    if self.enabled and (cycle % 4 == 0 or self.traj is not None
                                         or self.jog_s is not None):
                        sw = [ds402.statusword(n) for n in self.nodes]
                    self.step(q, sw)
            except Exception as e:      # CAN timeout etc.
                self.get_logger().error(f"CAN error: {e}", throttle_duration_sec=2.0)
                if self.enabled:
                    self.trip(f"CAN error: {e}")
                time.sleep(0.2)
                continue
            now = time.time()
            dt = now - self.t_prev
            if dt > 0:
                self.qd = [(a - b) / dt for a, b in zip(q, self.q_prev)]
            self.q, self.q_prev, self.t_prev = q, q, now
            self.publish(q)
            cycle += 1
            time.sleep(max(0.0, period - (time.time() - t_start)))

    def step(self, q, sw):
        if sw is not None:
            for i, s in enumerate(sw):
                if ds402.is_fault(s) or not ds402.is_enabled(s):
                    self.trip(f"joint {self.joint_names[i]} statusword 0x{s:04X}"
                              f"{' FAULT' if ds402.is_fault(s) else ' not enabled'}")
                    return
        with self.state_lock:
            traj, t0 = self.traj, self.traj_t0
        if self.enabled and traj is None:
            self.jog_step(q)
        else:
            self.jog_idle()
        if not self.enabled or traj is None:
            return
        t = time.time() - t0
        q_now_des = traj.sample(t)
        q_tgt = traj.sample(t + self.lookahead)
        self.trace_row(t, q, q_now_des, q_tgt, sw)
        errs = [abs(a - b) for a, b in zip(q, q_now_des)]
        err = max(errs)
        if err > self.path_tol:
            self.hold(q)
            self.report(t, q, q_now_des, sw)
            j = errs.index(err)
            self.finish(FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                        f"{self.joint_names[j]} tracking error {err / DEG:.1f} deg > "
                        f"{self.path_tol / DEG:.1f} (trace: {self.trace_path})")
            return
        for i, n in enumerate(self.nodes):
            v = min(self.max_vel, max(0.2 * DEG, abs(q_tgt[i] - q[i]) / self.lookahead))
            ds402.send_target(n, self.to_mdeg(i, q_tgt[i]), v / DEG * 1000.0)
        if t >= traj.duration:
            err_goal = max(abs(a - b) for a, b in zip(q, traj.q[-1]))
            if err_goal < self.goal_tol:
                self.finish(FollowJointTrajectory.Result.SUCCESSFUL, "")
            elif t > traj.duration + self.goal_time:
                self.hold(q)
                self.finish(FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                            f"goal error {err_goal / DEG:.2f} deg")

    # ---------- diagnostics ----------
    def trace_open(self):
        self.trace_path = os.path.expanduser(
            time.strftime("~/.ros/lwa4p_trace_%Y%m%d_%H%M%S.csv"))
        self.trace = open(self.trace_path, "w")
        cols = ["t", "cycle_dt"]
        for n in self.joint_names:
            j = n.replace("_joint", "")
            cols += [f"{j}_des", f"{j}_tgt", f"{j}_act", f"{j}_sw"]
        self.trace.write(",".join(cols) + "\n")
        self.trace_last = time.time()

    def trace_row(self, t, q, q_des, q_tgt, sw):
        if self.trace is None:
            return
        now = time.time()
        row = [f"{t:.3f}", f"{now - self.trace_last:.3f}"]
        self.trace_last = now
        for i in range(len(q)):
            row += [f"{q_des[i] / DEG:.2f}", f"{q_tgt[i] / DEG:.2f}", f"{q[i] / DEG:.2f}",
                    f"0x{sw[i]:04X}" if sw else ""]
        self.trace.write(",".join(row) + "\n")

    def trace_close(self):
        if self.trace is not None:
            self.trace.close()
            self.trace = None

    def report(self, t, q, q_des, sw):
        lines = [f"abort at t={t:.2f} s:"]
        for i, n in enumerate(self.joint_names):
            s = f"0x{sw[i]:04X}" if sw else "?"
            flags = []
            if sw and sw[i] & (1 << 13):
                flags.append("FOLLOWING ERROR")
            if sw and sw[i] & (1 << 12):
                flags.append("setpoint ack")
            lines.append(f"  {n}: desired {q_des[i] / DEG:8.2f}  actual {q[i] / DEG:8.2f}  "
                         f"err {(q[i] - q_des[i]) / DEG:+6.2f} deg  sw {s} {' '.join(flags)}")
        self.get_logger().warn("\n".join(lines))

    def hold(self, q):
        for i, n in enumerate(self.nodes):
            try:
                ds402.send_target(n, self.to_mdeg(i, q[i]), self.max_vel / DEG * 1000.0)
            except Exception:
                pass

    def trip(self, why):
        self.get_logger().error(f"{why} -> disabling all drives")
        for n in self.nodes:
            ds402.disable(n)
        self.enabled = False
        self.fault = why
        self.finish(FollowJointTrajectory.Result.INVALID_GOAL, why)

    def finish(self, code, msg):
        self.trace_close()
        with self.state_lock:
            if self.traj is not None:
                self.traj = None
                self.traj_result = (code, msg)

    def publish(self, q):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = self.joint_names
        m.position = q
        m.velocity = self.qd
        self.pub.publish(m)

    # ---------- services ----------
    def srv_reset(self, req, res):
        with self.can_lock:
            states = []
            for i, n in enumerate(self.nodes):
                sw = ds402.bring_up(n)
                st = ds402.ready_state(sw)
                err = ds402.last_error(n)
                states.append(f"{self.joint_names[i]}: 0x{sw:04X} {st or 'NOT READY'}"
                              + (f" (last error 0x{err:04X})" if err and not st else ""))
            ok = all(ds402.ready_state(ds402.statusword(n)) for n in self.nodes)
        self.fault = "" if ok else self.fault
        res.success, res.message = ok, "; ".join(states)
        return res

    def srv_enable(self, req, res):
        r = self.srv_reset(req, Trigger.Response())
        if not r.success:
            res.success, res.message = False, "not ready (motor power? E-stop?): " + r.message
            return res
        try:
            with self.can_lock:
                for n in self.nodes:
                    ds402.setup_pp(n, self.accel_mdeg)
                order = sorted(range(len(self.nodes)), key=lambda i: self.node_ids[i])
                for i in order:
                    ds402.enable(self.nodes[i])
                    time.sleep(self.enable_settle)
                    if not ds402.commutate(self.nodes[i]):
                        raise RuntimeError(f"{self.joint_names[i]} did not move during the "
                                           f"commutation check (+0.5 deg and back)")
                    sw = ds402.statusword(self.nodes[i])
                    if not ds402.is_enabled(sw):
                        raise RuntimeError(f"{self.joint_names[i]} did not stay enabled "
                                           f"(statusword 0x{sw:04X}, last error "
                                           f"0x{ds402.last_error(self.nodes[i]) or 0:04X})")
                    self.get_logger().info(f"{self.joint_names[i]} enabled")
                self.enabled = True
                self.fault = ""
            res.success, res.message = True, "all joints enabled, holding position"
        except Exception as e:
            with self.can_lock:
                for n in self.nodes:
                    ds402.disable(n)
                self.enabled = False
            res.success, res.message = False, f"enable failed: {e}"
        return res

    def srv_disable(self, req, res):
        with self.can_lock:
            self.finish(FollowJointTrajectory.Result.INVALID_GOAL, "disabled")
            for n in self.nodes:
                ds402.disable(n)
            self.enabled = False
        res.success, res.message = True, "all joints disabled (brakes closed)"
        return res

    # ---------- jogging ----------
    # ~/jog takes control_msgs/JointJog: velocities (rad/s) jog while messages keep coming,
    # displacements (rad) move a step at the given velocity. A velocity message replaces the
    # whole command (unlisted joints stop, pending steps are dropped), so all zeros = stop.
    # The loop runs its own setpoint per joint with an acceleration ramp and feeds it to the
    # drives like a trajectory. Jogging and trajectories exclude each other.
    def jog_init(self, cb):
        p = self.declare_parameter
        self.jog_max_vel = min(self.max_vel, p("jog_max_velocity_deg_s", 15.0).value * DEG)
        self.jog_accel = p("jog_acceleration_deg_s2", 20.0).value * DEG
        self.jog_timeout = p("jog_timeout_s", 0.25).value
        n = len(self.nodes)
        # written by on_jog, read by the loop (state_lock)
        self.jog_vd = [0.0] * n
        self.jog_stamp = 0.0
        self.jog_steps = [0.0] * n
        self.jog_step_vel = [self.jog_max_vel] * n
        self.jog_cancel = False
        self.jog_blocked = False
        # loop only
        self.jog_s = None               # setpoint per joint, None = not jogging
        self.jog_v = [0.0] * n
        self.jog_goal = [None] * n      # step target per joint
        self.jog_moving = [False] * n
        self.jog_t = 0.0
        self.create_subscription(JointJog, "~/jog", self.on_jog, 10, callback_group=cb)

    def jog_reject(self, why):
        self.get_logger().warn(f"ignoring jog: {why}", throttle_duration_sec=2.0)

    def on_jog(self, msg):
        warn = self.jog_reject
        if not self.enabled:
            return warn("drives not enabled (call ~/enable)")
        if self.traj is not None:
            return warn("trajectory running")
        try:
            idx = [self.joint_names.index(j) for j in msg.joint_names]
        except ValueError:
            return warn(f"unknown joint in {list(msg.joint_names)}")
        vel = list(msg.velocities)
        if msg.displacements:
            if len(msg.displacements) != len(idx) or vel and len(vel) != len(idx):
                return warn("joint_names, displacements and velocities differ in length")
            with self.state_lock:
                if self.jog_blocked:
                    return warn("stopped after a tracking error - send a stop (all zero velocities)")
                for k, i in enumerate(idx):
                    if msg.displacements[k]:
                        self.jog_steps[i] += msg.displacements[k]
                        v = abs(vel[k]) if vel and vel[k] else self.jog_max_vel
                        self.jog_step_vel[i] = min(self.jog_max_vel, v)
            return
        if len(vel) != len(idx):
            return warn("joint_names and velocities differ in length")
        vd = [0.0] * len(self.nodes)
        for k, i in enumerate(idx):
            vd[i] = max(-self.jog_max_vel, min(self.jog_max_vel, vel[k]))
        with self.state_lock:
            if self.jog_blocked:
                if any(vd):
                    return warn("stopped after a tracking error - release and jog again")
                self.jog_blocked = False
            self.jog_vd = vd
            self.jog_stamp = time.time()
            self.jog_steps = [0.0] * len(self.nodes)
            self.jog_cancel = True

    def jog_idle(self):
        self.jog_s = None
        self.jog_moving = [False] * len(self.nodes)
        with self.state_lock:
            self.jog_vd = [0.0] * len(self.nodes)
            self.jog_steps = [0.0] * len(self.nodes)
            self.jog_blocked = False

    def jog_step(self, q):
        now = time.time()
        n = len(q)
        with self.state_lock:
            vd = self.jog_vd if now - self.jog_stamp < self.jog_timeout else [0.0] * n
            steps, self.jog_steps = self.jog_steps, [0.0] * n
            step_vel = list(self.jog_step_vel)
            cancel, self.jog_cancel = self.jog_cancel, False
        if self.jog_s is None:
            if not any(vd) and not any(steps):
                return
            self.jog_s, self.jog_v = list(q), [0.0] * n
            self.jog_goal, self.jog_moving = [None] * n, [False] * n
            self.jog_t = now - 1.0 / self.rate
        dt = min(now - self.jog_t, 0.2)
        self.jog_t = now
        s, v, a = self.jog_s, self.jog_v, self.jog_accel
        err = max(abs(x - y) for x, y in zip(q, s))
        if err > self.path_tol:
            self.hold(q)
            self.jog_s = None
            with self.state_lock:
                self.jog_vd, self.jog_blocked = [0.0] * n, True
            self.get_logger().error(f"jog stopped: tracking error {err / DEG:.1f} deg > "
                                    f"{self.path_tol / DEG:.1f} (blocked? collision?)")
            return
        moving = [False] * n
        for i in range(n):
            lo, hi = self.limits[i]
            lo, hi = lo + self.limit_margin, hi - self.limit_margin
            if cancel:
                self.jog_goal[i] = None
            if steps[i]:
                base = s[i] if self.jog_goal[i] is None else self.jog_goal[i]
                self.jog_goal[i] = min(hi, max(lo, base + steps[i]))
            goal = self.jog_goal[i]
            if goal is not None:
                e = goal - s[i]
                want = math.copysign(min(step_vel[i], math.sqrt(2 * a * abs(e))), e)
            else:
                want = vd[i]
            vi = v[i] + max(-a * dt, min(a * dt, want - v[i]))
            if goal is not None:        # brake in time for the step target ...
                cap = math.sqrt(2 * a * abs(goal - s[i]))
                vi = max(-cap, min(cap, vi)) if (goal - s[i]) * vi > 0 else vi
            if vi > 0:                  # ... and for the drive limits
                vi = min(vi, math.sqrt(2 * a * max(0.0, hi - s[i])))
            elif vi < 0:
                vi = max(vi, -math.sqrt(2 * a * max(0.0, s[i] - lo)))
            si = s[i] + vi * dt
            si = min(si, max(hi, s[i])) if vi > 0 else max(si, min(lo, s[i]))
            if goal is not None and ((goal - si) * (goal - s[i]) <= 0 or abs(goal - si) < 1e-5):
                si, vi, self.jog_goal[i] = goal, 0.0, None
            s[i], v[i] = si, vi
            moving[i] = vi != 0.0 or self.jog_goal[i] is not None
        for i, node in enumerate(self.nodes):
            if moving[i] or self.jog_moving[i]:      # once more after stopping: final target
                # lead the setpoint, but by no more than its braking distance (in steps of dt):
                # the target then never moves back while braking, so the drive does not overshoot
                lead = max(0.0, min(abs(v[i]) * self.lookahead,
                                    v[i] * v[i] / (2 * a) - abs(v[i]) * dt / 2))
                lo, hi = self.limits[i]
                lo, hi = min(lo + self.limit_margin, s[i]), max(hi - self.limit_margin, s[i])
                if self.jog_goal[i] is not None:        # never past the step target
                    lo, hi = sorted((s[i], self.jog_goal[i]))
                tgt = min(hi, max(lo, s[i] + math.copysign(lead, v[i])))
                # setpoint speed plus a catch-up term for how far the drive lags behind it
                lag = (s[i] - q[i]) * (1.0 if v[i] > 0 else -1.0) if v[i] else abs(s[i] - q[i])
                vel = min(self.max_vel, max(0.2 * DEG, abs(v[i]) + lag / self.lookahead))
                ds402.send_target(node, self.to_mdeg(i, tgt), vel / DEG * 1000.0)
        self.jog_moving = moving
        if not any(moving) and not any(vd):
            self.jog_s = None

    # ---------- action ----------
    def on_goal(self, goal):
        ok, why = self.check_goal(goal.trajectory)
        if not ok:
            self.get_logger().warn(f"rejecting trajectory: {why}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def check_goal(self, jt):
        if not self.enabled:
            return False, "drives not enabled (call ~/enable)"
        if self.jog_s is not None:
            return False, "arm is being jogged"
        if sorted(jt.joint_names) != sorted(self.joint_names):
            return False, f"joint names {list(jt.joint_names)} != {self.joint_names}"
        if not jt.points:
            return False, "empty trajectory"
        return True, ""

    def build(self, jt):
        idx = [list(jt.joint_names).index(n) for n in self.joint_names]
        times, qs, vs = [], [], []
        has_vel = all(len(pt.velocities) == len(idx) for pt in jt.points)
        for pt in jt.points:
            times.append(pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9)
            qs.append([pt.positions[k] for k in idx])
            if has_vel:
                vs.append([pt.velocities[k] for k in idx])
        q0 = list(self.q)
        if times[0] > 1e-3:
            times.insert(0, 0.0)
            qs.insert(0, q0)
            if has_vel:
                vs.insert(0, [0.0] * len(q0))
        else:
            err = max(abs(a - b) for a, b in zip(qs[0], q0))
            if err > self.start_tol:
                return None, f"start {err / DEG:.1f} deg away from current position"
        for k in range(len(times)):
            for i in range(len(q0)):
                lo, hi = self.limits[i]
                if not lo + self.limit_margin <= qs[k][i] <= hi - self.limit_margin:
                    return None, (f"{self.joint_names[i]} = {qs[k][i] / DEG:.1f} deg "
                                  f"outside drive limits")
            if k and times[k] <= times[k - 1]:
                return None, "time_from_start not increasing"
            if k:
                v = max(abs(a - b) for a, b in zip(qs[k], qs[k - 1])) / (times[k] - times[k - 1])
                if v > self.max_vel * 1.05:
                    return None, (f"segment velocity {v / DEG:.1f} deg/s > max "
                                  f"{self.max_vel / DEG:.1f} (scale the trajectory down)")
        return Trajectory(times, qs, vs if has_vel else None), ""

    def execute(self, gh):
        res = FollowJointTrajectory.Result()
        traj, why = self.build(gh.request.trajectory)
        if traj is None:
            self.get_logger().warn(f"aborting trajectory: {why}")
            res.error_code, res.error_string = res.INVALID_GOAL, why
            gh.abort()
            return res
        with self.can_lock:
            self.trace_open()
        with self.state_lock:
            self.traj_result = None
            self.traj_t0 = time.time()
            self.traj = traj
        self.get_logger().info(
            f"executing trajectory ({len(traj.t)} points, {traj.duration:.1f} s): " + ", ".join(
                f"{n.replace('_joint', '')} {a / DEG:.1f}->{b / DEG:.1f}"
                for n, a, b in zip(self.joint_names, traj.q[0], traj.q[-1]) if abs(a - b) > 0.1 * DEG))
        fb = FollowJointTrajectory.Feedback()
        fb.joint_names = self.joint_names
        while rclpy.ok():
            with self.state_lock:
                result = self.traj_result
            if result is not None:
                break
            if gh.is_cancel_requested:
                with self.can_lock:
                    self.hold(self.q)
                self.finish(res.SUCCESSFUL, "canceled")
                gh.canceled()
                self.get_logger().info("trajectory canceled, holding")
                return res
            t = time.time() - self.traj_t0
            fb.desired.positions = traj.sample(t)
            fb.actual.positions = list(self.q)
            fb.error.positions = [a - d for a, d in zip(fb.actual.positions, fb.desired.positions)]
            gh.publish_feedback(fb)
            time.sleep(0.05)
        code, msg = result
        res.error_code, res.error_string = code, msg
        if code == res.SUCCESSFUL:
            gh.succeed()
            self.get_logger().info("trajectory done")
        else:
            gh.abort()
            self.get_logger().warn(f"trajectory aborted: {msg}")
        return res

    def shutdown(self):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        self.running = False
        self.thread.join(timeout=1.0)
        if self.can_lock.acquire(timeout=2.0):
            self.can_lock.release()
        for n in self.nodes:
            ds402.disable(n)
        self.get_logger().info("all drives disabled")
        self.net.disconnect()


def main():
    rclpy.init()
    node = Lwa4pDriver()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

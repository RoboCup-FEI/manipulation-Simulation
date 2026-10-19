import bisect
import math
import os
import signal
import threading
import time

import canopen
import rclpy
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
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
        self.effort_scale = list(p("effort_scale", [1.0] * 6).value)
        self.effort_alpha = p("effort_alpha", 0.5).value
        diag_rate = p("diagnostics_rate_hz", 1.0).value
        log_period = p("status_log_period_s", 2.0).value
        self.temp_warn = p("temperature_warn_c", 60.0).value
        self.temp_error = p("temperature_error_c", 75.0).value
        controller = p("controller_name", "arm_controller").value
        assert len(self.node_ids) == len(self.joint_names) == len(self.signs) == len(self.offsets)
        assert len(self.effort_scale) == len(self.node_ids)

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
            self.rated_a = [ds402.rated_current(n) / 1000.0 for n in self.nodes]
            self.q =[self.to_rad(i, ds402.position(n)) for i, n in enumerate(self.nodes)]


        self.state_lock = threading.Lock()
        self.enabled = False
        self.fault = ""
        self.traj = None
        self.traj_t0 = 0.0
        self.traj_result = None
        self.trace, self.trace_path = None, ""
        self.q_prev, self.t_prev = list(self.q), time.time()
        self.qd = [0.0] * len(self.nodes)
        self.torque = [None] * len(self.nodes)      # smoothed 0x6077, per mille of rated
        self.tq_stats = [[0.0, 0.0, 0, 0] for _ in self.nodes]   # sum, sum |x|, n, peak since last diagnostics

        # ---- ROS interfaces ----
        cb = ReentrantCallbackGroup()
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(1.0 / diag_rate, self.publish_diagnostics, callback_group=cb)
        self.last_diag = None
        if log_period > 0:
            self.create_timer(log_period, self.log_status, callback_group=cb)
        self.create_service(Trigger, "~/enable", self.srv_enable, callback_group=cb)
        self.create_service(Trigger, "~/disable", self.srv_disable, callback_group=cb)
        self.create_service(Trigger, "~/reset", self.srv_reset, callback_group=cb)
        self.create_service(Trigger, "~/diagnose", self.srv_diagnose, callback_group=cb)
        self.action = ActionServer(
            self, FollowJointTrajectory, f"{controller}/follow_joint_trajectory",
            execute_callback=self.execute, goal_callback=self.on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=cb)

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
                    if self.enabled and (cycle % 4 == 0 or self.traj is not None):
                        sw = [ds402.statusword(n) for n in self.nodes]
                    self.step(q, sw)
                    k = cycle % len(self.nodes)     # one torque read per cycle keeps the loop short
                    tq = ds402.torque(self.nodes[k])
            except Exception as e:      # CAN timeout etc.
                self.get_logger().error(f"CAN error: {e}", throttle_duration_sec=2.0)
                if self.enabled:
                    self.trip(f"CAN error: {e}")
                time.sleep(0.2)
                continue
            if tq is not None:
                self.update_torque(k, tq)
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
            bad = [i for i, s in enumerate(sw) if ds402.is_fault(s) or not ds402.is_enabled(s)]
            if bad:
                self.trip("drive fault", [(i, sw[i]) for i in bad])
                return
        with self.state_lock:
            traj, t0 = self.traj, self.traj_t0
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

    def joint_report(self, i, sw=None, torque=()):
        n = self.nodes[i]
        sw = ds402.statusword(n) if sw is None else sw
        state = ds402.ready_state(sw) or ("FAULT" if ds402.is_fault(sw) else "not ready")
        s = f"{self.joint_names[i]}: statusword 0x{sw:04X} {state}"
        t = ds402.temperature(n)
        if t is not None:
            s += f", {t:.1f} C"
        torque = [x for x in torque if x is not None]
        if torque:      # per mille samples; current estimated from mean |torque| x rated current
            mean = sum(torque) / len(torque)
            mean_abs = sum(abs(x) for x in torque) / len(torque)
            peak = max(torque, key=abs)
            s += (f", torque {mean / 10:+.0f}% (peak {peak / 10:+.0f}%) of rated, "
                  f"~{mean_abs / 1000 * self.rated_a[i]:.2f} A of {self.rated_a[i]:.2f} A rated")
        errs = ds402.error_history(n)
        if errs:
            s += ", errors " + ", ".join(f"0x{e:04X} {ds402.error_name(e)}" for e in errs)
        return s

    def temperatures(self):
        ts = [ds402.temperature(n) for n in self.nodes]
        return " ".join(f"{name.replace('_joint', '')}={'?' if t is None else f'{t:.1f}'}"
                        for name, t in zip(self.joint_names, ts)) + " C"

    def hold(self, q):
        for i, n in enumerate(self.nodes):
            try:
                ds402.send_target(n, self.to_mdeg(i, q[i]), self.max_vel / DEG * 1000.0)
            except Exception:
                pass

    def trip(self, why, bad=()):
        for n in self.nodes:
            ds402.disable(n)
        self.enabled = False
        if bad:     # details are read after the stop; the error history survives the disable
            try:
                why = "; ".join(self.joint_report(i, s) for i, s in bad)
                self.get_logger().error(f"temperatures at trip: {self.temperatures()}")
            except Exception:
                why = "; ".join(f"{self.joint_names[i]} statusword 0x{s:04X}" for i, s in bad)
        self.get_logger().error(f"{why} -> all drives disabled")
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
        m.effort = [self.signs[i] * (t or 0.0) / 1000.0 * self.effort_scale[i]
                    for i, t in enumerate(self.torque)]
        self.pub.publish(m)

    def update_torque(self, i, tq):
        prev = self.torque[i]
        self.torque[i] = tq if prev is None else prev + self.effort_alpha * (tq - prev)
        st = self.tq_stats[i]
        st[0] += tq
        st[1] += abs(tq)
        st[2] += 1
        if abs(tq) > abs(st[3]):
            st[3] = tq

    def publish_diagnostics(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        drv = DiagnosticStatus(name="lwa4p_driver: drives", hardware_id=self.channel)
        if self.enabled:
            drv.level = DiagnosticStatus.OK
            drv.message = "enabled" + (", executing trajectory" if self.traj is not None else "")
        elif self.fault:
            drv.level, drv.message = DiagnosticStatus.WARN, f"disabled after trip: {self.fault}"
        else:
            drv.level, drv.message = DiagnosticStatus.OK, "disabled"
        arr.status.append(drv)
        arr.status += [self.joint_diagnostics(i) for i in range(len(self.nodes))]
        self.diag_pub.publish(arr)
        self.last_diag = arr

    def log_status(self):
        arr = self.last_diag
        if arr is None:
            return
        drv, joints = arr.status[0], arr.status[1:]
        lines = [f"status: drives {drv.message}"]
        for s in joints:
            v = {kv.key: kv.value for kv in s.values}
            name = s.name.split(": ", 1)[-1].replace("_joint", "")
            if not v:                       # no response from the drive
                lines.append(f"  {name}  {s.message}")
                continue
            line = f"  {name}  {v['position_deg']:>8} deg  {v['temperature_c']:>5} C"
            if "torque_mean_pct" in v:
                line += (f"  torque {v['torque_mean_pct']:>6}% (peak {v['torque_peak_pct']:>6}%)"
                         f"  ~{v['current_est_a']} A")
            lines.append(f"{line}  {s.message}")
        level = max(s.level for s in arr.status)
        log = self.get_logger()
        out = (log.error if level >= DiagnosticStatus.ERROR else
               log.warn if level >= DiagnosticStatus.WARN else log.info)
        out("\n".join(lines))

    def joint_diagnostics(self, i):
        n = self.nodes[i]
        st = DiagnosticStatus(name=f"lwa4p_driver: {self.joint_names[i]}", hardware_id=f"CANopen node {n.id}")
        stats, self.tq_stats[i] = self.tq_stats[i], [0.0, 0.0, 0, 0]
        try:
            with self.can_lock:     # one joint at a time so the control loop is not held up
                sw = ds402.statusword(n)
                temp = ds402.temperature(n)
                errs = ds402.error_history(n)
        except Exception as e:
            st.level, st.message = DiagnosticStatus.ERROR, f"no response: {e}"
            return st
        state = ds402.ready_state(sw) or ("FAULT" if ds402.is_fault(sw) else "not ready")
        st.level = DiagnosticStatus.ERROR if ds402.is_fault(sw) else DiagnosticStatus.OK
        st.message = state
        if temp is not None and temp >= self.temp_error:
            st.level = DiagnosticStatus.ERROR
            st.message += f", temperature {temp:.1f} C >= {self.temp_error:.0f} C"
        elif temp is not None and temp >= self.temp_warn:
            st.level = max(st.level, DiagnosticStatus.WARN)
            st.message += f", temperature {temp:.1f} C >= {self.temp_warn:.0f} C"
        if errs and ds402.is_fault(sw):
            st.message += f", error 0x{errs[0]:04X} {ds402.error_name(errs[0])}"
        vals = {"state": state, "statusword": f"0x{sw:04X}", "position_deg": f"{self.q[i] / DEG:.2f}",
                "temperature_c": "?" if temp is None else f"{temp:.1f}",
                "rated_current_a": f"{self.rated_a[i]:.2f}",
                "error_history": ", ".join(f"0x{e:04X} {ds402.error_name(e)}" for e in errs) or "none"}
        total, total_abs, cnt, peak = stats
        if cnt:     # torque in % of rated, joint sign applied; current estimated from mean |torque|
            sign = self.signs[i]
            vals.update(torque_mean_pct=f"{sign * total / cnt / 10:+.1f}",
                        torque_peak_pct=f"{sign * peak / 10:+.1f}",
                        current_est_a=f"{total_abs / cnt / 1000 * self.rated_a[i]:.2f}",
                        torque_samples=str(cnt))
        st.values = [KeyValue(key=k, value=v) for k, v in vals.items()]
        return st

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
                                           f"commutation check (+0.5 deg and back) - "
                                           f"{self.joint_report(i, torque=[ds402.torque(self.nodes[i])])}")
                    sw = ds402.statusword(self.nodes[i])
                    if not ds402.is_enabled(sw):
                        raise RuntimeError(f"{self.joint_names[i]} did not stay enabled - "
                                           f"{self.joint_report(i, sw)}")
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
            self.get_logger().error(res.message)
        return res

    def srv_diagnose(self, req, res):
        torque = [[] for _ in self.nodes]
        for _ in range(10):     # torque is noisy; lock per round so a running trajectory is not stalled
            with self.can_lock:
                for i, n in enumerate(self.nodes):
                    torque[i].append(ds402.torque(n))
            time.sleep(0.02)
        with self.can_lock:
            lines = [self.joint_report(i, torque=torque[i]) for i in range(len(self.nodes))]
        if self.fault:
            lines.insert(0, f"last trip: {self.fault}")
        res.success, res.message = True, "\n".join(lines)
        return res

    def srv_disable(self, req, res):
        with self.can_lock:
            self.finish(FollowJointTrajectory.Result.INVALID_GOAL, "disabled")
            for n in self.nodes:
                ds402.disable(n)
            self.enabled = False
        res.success, res.message = True, "all joints disabled (brakes closed)"
        return res

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

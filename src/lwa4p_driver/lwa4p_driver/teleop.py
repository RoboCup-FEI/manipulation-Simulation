import math
import os
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from control_msgs.action import GripperCommand
from control_msgs.msg import JointJog
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_srvs.srv import Trigger

DEG = math.pi / 180.0

# key -> (joint index, direction)
KEYS = {"q": (0, 1), "a": (0, -1), "w": (1, 1), "s": (1, -1), "e": (2, 1), "d": (2, -1),
        "r": (3, 1), "f": (3, -1), "t": (4, 1), "g": (4, -1), "y": (5, 1), "h": (5, -1)}

HELP = """
LWA 4P jog - keyboard
  q/a w/s e/d r/f t/g y/h   joint 1..6  + / -
  , .     step smaller / larger         - +     speed down / up
  m       step <-> hold mode            space   STOP
  o c     gripper open / grasp          p       print joint positions
  X       disable drives (brakes)       ?       this help      Ctrl+C quit
"""


class Teleop(Node):
    def __init__(self):
        super().__init__("teleop")
        p = self.declare_parameter
        self.joint_names = list(p("joint_names", [f"arm_{i}_joint" for i in range(1, 7)]).value)
        jog_topic = p("jog_topic", "/lwa4p_driver/jog").value
        self.max_vel = p("max_velocity_deg_s", 15.0).value * DEG
        self.speeds = list(p("speed_levels", [0.1, 0.25, 0.5, 1.0]).value)
        self.speed_i = min(len(self.speeds) - 1, max(0, p("speed_level", 1).value))
        self.steps = [s * DEG for s in p("step_sizes_deg", [0.1, 0.5, 1.0, 2.0, 5.0]).value]
        self.step_i = min(len(self.steps) - 1, max(0, p("step_size", 2).value))
        rate = p("rate_hz", 20.0).value
        keyboard = p("keyboard", True).value
        self.key_hold = p("key_hold_s", 0.7).value
        self.key_repeat = p("key_repeat_s", 0.1).value
        # gamepad, see config/teleop.yaml
        self.joy_timeout = p("joy_timeout_s", 0.5).value
        self.deadman = p("deadman_button", 4).value
        self.joint_axes = list(p("joint_axes", [0, 1, 4, 3, -1, 6]).value)
        self.joint_signs = list(p("joint_axis_signs", [1.0] * 6).value)
        self.trigger_joint = p("trigger_joint", 5).value - 1
        self.trigger_axes = list(p("trigger_axes", [5, 2]).value)       # + / -
        self.trigger_rest = p("trigger_rest", 1.0).value
        self.trigger_pressed = p("trigger_pressed", -1.0).value
        self.deadzone = p("deadzone", 0.15).value
        self.expo = p("response_exponent", 2.0).value
        self.speed_axis = p("speed_axis", 7).value
        self.btn_open = p("open_button", 0).value
        self.btn_grasp = p("grasp_button", 1).value
        self.btn_disable = p("disable_button", 6).value
        self.open_gap = p("gripper_open_m", 0.085).value
        self.grasp_force = p("grasp_force_n", 35.0).value
        assert len(self.joint_axes) == len(self.joint_signs) == len(self.joint_names)

        self.lock = threading.Lock()
        self.joy, self.joy_t = None, 0.0
        self.trigger_seen = set()       # triggers report 0.0 until first touched
        self.armed = False              # deadman held and inputs were neutral when pressed
        self.mode = "step"
        self.held = {}                  # hold mode: joint -> (direction, time of last key)
        self.key_t = {}
        self.hold_active = False
        self.q = None

        self.pub = self.create_publisher(JointJog, jog_topic, 10)
        self.create_subscription(Joy, "joy", self.on_joy, 10)
        self.create_subscription(JointState, "joint_states", self.on_joint_states, 10)
        self.grip = ActionClient(self, GripperCommand, "gripper_controller/gripper_cmd")
        self.disable_srv = self.create_client(Trigger, "/lwa4p_driver/disable")
        self.create_timer(1.0 / rate, self.tick)

        self.running = True
        self.tty = None
        if keyboard and sys.stdin.isatty():
            self.tty = termios.tcgetattr(sys.stdin.fileno())
            threading.Thread(target=self.keyboard, daemon=True).start()
            print(HELP)
        elif keyboard:
            self.get_logger().info("stdin is not a terminal - keyboard off, gamepad only")
        self.get_logger().info(
            f"publishing on {jog_topic}; gamepad: hold button {self.deadman} to jog; {self.status()}")

    # ---------- helpers ----------
    def status(self):
        v = self.max_vel * self.speeds[self.speed_i]
        return (f"speed {self.speeds[self.speed_i] * 100:.0f} % ({v / DEG:.1f} deg/s), "
                f"step {self.steps[self.step_i] / DEG:g} deg, keyboard mode {self.mode}")

    def speed(self):
        return self.max_vel * self.speeds[self.speed_i]

    def change(self, what, d):
        if what == "speed":
            self.speed_i = max(0, min(len(self.speeds) - 1, self.speed_i + d))
        else:
            self.step_i = max(0, min(len(self.steps) - 1, self.step_i + d))
        self.get_logger().info(self.status())

    def publish(self, velocities=None, joint=None, step=0.0):
        m = JointJog()
        m.header.stamp = self.get_clock().now().to_msg()
        if joint is None:
            m.joint_names = self.joint_names
            m.velocities = [float(v) for v in velocities]
        else:
            m.joint_names = [self.joint_names[joint]]
            m.displacements = [float(step)]
            m.velocities = [self.speed()]
        self.pub.publish(m)

    def stop(self):
        with self.lock:
            self.held.clear()
        self.publish([0.0] * len(self.joint_names))

    def on_joint_states(self, m):
        if all(j in m.name for j in self.joint_names):
            self.q = [m.position[list(m.name).index(j)] for j in self.joint_names]

    # ---------- gripper / drives ----------
    def gripper(self, gap, force):
        if not self.grip.server_is_ready():
            self.get_logger().warn("gripper action server not available")
            return
        goal = GripperCommand.Goal()
        goal.command.position, goal.command.max_effort = gap, force
        self.grip.send_goal_async(goal).add_done_callback(self.on_gripper_goal)

    def on_gripper_goal(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn("gripper goal rejected")
            return
        gh.get_result_async().add_done_callback(lambda f: self.get_logger().info(
            f"gripper: gap {f.result().result.position * 1000:.1f} mm, "
            f"{f.result().result.effort:.1f} N"))

    def disable(self):
        self.stop()
        if not self.disable_srv.service_is_ready():
            self.get_logger().warn("/lwa4p_driver/disable not available")
            return
        self.disable_srv.call_async(Trigger.Request()).add_done_callback(
            lambda f: self.get_logger().warn(f"disable: {f.result().message}"))

    # ---------- gamepad ----------
    @staticmethod
    def axis(m, a):
        return m.axes[a] if m is not None and 0 <= a < len(m.axes) else 0.0

    @staticmethod
    def button(m, b):
        return m is not None and 0 <= b < len(m.buttons) and m.buttons[b] == 1

    def shape(self, x):
        if abs(x) < self.deadzone:
            return 0.0
        return math.copysign(min(1.0, (abs(x) - self.deadzone) / (1.0 - self.deadzone)) ** self.expo, x)

    def trigger(self, m, a):
        raw = self.axis(m, a)
        if abs(raw - self.trigger_rest) < 0.05:
            self.trigger_seen.add(a)
        if a not in self.trigger_seen:
            return 0.0
        return self.shape(max(0.0, min(1.0, (raw - self.trigger_rest)
                                       / (self.trigger_pressed - self.trigger_rest))))

    def joy_velocities(self, m):
        v = [self.joint_signs[i] * self.shape(self.axis(m, a)) if a >= 0 else 0.0
             for i, a in enumerate(self.joint_axes)]
        if 0 <= self.trigger_joint < len(v):
            v[self.trigger_joint] += (self.trigger(m, self.trigger_axes[0])
                                      - self.trigger(m, self.trigger_axes[1]))
        return [max(-1.0, min(1.0, x)) * self.speed() for x in v]

    def on_joy(self, m):
        prev, self.joy, self.joy_t = self.joy, m, time.time()
        for a in self.trigger_axes:
            self.trigger(m, a)

        def pressed(b):
            return self.button(m, b) and not self.button(prev, b)

        if pressed(self.btn_disable):
            self.disable()
        if pressed(self.btn_open):
            self.gripper(self.open_gap, 0.0)
        if pressed(self.btn_grasp):
            self.gripper(0.0, self.grasp_force)
        s, ps = self.axis(m, self.speed_axis), self.axis(prev, self.speed_axis)
        if s > 0.5 >= ps:
            self.change("speed", 1)
        elif s < -0.5 <= ps:
            self.change("speed", -1)

    # ---------- periodic ----------
    def tick(self):
        now = time.time()
        m = self.joy
        if m is not None and now - self.joy_t < self.joy_timeout and self.button(m, self.deadman):
            v = self.joy_velocities(m)
            if not self.armed and any(v):
                self.get_logger().warn("release the sticks and triggers, then press the deadman again",
                                       throttle_duration_sec=2.0)
                return
            self.armed = True
            self.publish(v)
            return
        if self.armed:
            self.armed = False
            self.publish([0.0] * len(self.joint_names))
            if m is not None and now - self.joy_t >= self.joy_timeout and self.button(m, self.deadman):
                self.get_logger().warn("no /joy messages - stopped")
        with self.lock:
            v = [0.0] * len(self.joint_names)
            for i, (d, t) in list(self.held.items()):
                if now - t < self.key_hold:
                    v[i] = d * self.speed()
                else:
                    del self.held[i]
        if any(v):
            self.hold_active = True
            self.publish(v)
        elif self.hold_active:
            self.hold_active = False
            self.publish(v)

    # ---------- keyboard ----------
    def keyboard(self):
        fd = sys.stdin.fileno()
        tty.setcbreak(fd)
        while self.running and rclpy.ok():
            if not select.select([fd], [], [], 0.1)[0]:
                continue
            data = os.read(fd, 64).decode(errors="ignore")
            if data.startswith("\x1b"):     # arrows, F-keys: ignore
                continue
            for c in data:
                self.on_key(c)

    def on_key(self, c):
        now = time.time()
        if c in KEYS:
            i, d = KEYS[c]
            last, self.key_t[c] = self.key_t.get(c, 0.0), now
            if self.mode == "hold":
                with self.lock:
                    self.held[i] = (d, now)
            elif now - last > self.key_repeat:      # auto-repeat of a held key: one step only
                self.publish(joint=i, step=d * self.steps[self.step_i])
        elif c == " ":
            self.stop()
            self.get_logger().info("STOP")
        elif c in "+=":
            self.change("speed", 1)
        elif c in "-_":
            self.change("speed", -1)
        elif c in ".>":
            self.change("step", 1)
        elif c in ",<":
            self.change("step", -1)
        elif c == "m":
            self.stop()
            self.mode = "hold" if self.mode == "step" else "step"
            self.get_logger().info(self.status())
        elif c == "o":
            self.gripper(self.open_gap, 0.0)
        elif c == "c":
            self.gripper(0.0, self.grasp_force)
        elif c == "X":
            self.disable()
        elif c == "p":
            if self.q is None:
                self.get_logger().warn("no /joint_states yet")
            else:
                self.get_logger().info("joints [deg]: " + " ".join(f"{x / DEG:7.2f}" for x in self.q))
        elif c == "?":
            print(HELP + "  " + self.status())

    def close(self):
        self.running = False
        if self.tty is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.tty)


def main():
    rclpy.init()
    node = Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

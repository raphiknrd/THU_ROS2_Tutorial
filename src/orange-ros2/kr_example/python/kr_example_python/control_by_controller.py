import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from kr_msgs.msg import JogLinear
from griplink_interfaces.action import Grip, Release
from rclpy.action import ActionClient

class JogLinearJoyControl(Node):

    def __init__(self):
        super().__init__('jog_linear_joy_control')
        
        # Publisher für Bewegung
        self.publisher_ = self.create_publisher(JogLinear, "/kr/motion/jog_linear", 10)
        
        # Subscriber für Controller
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        
        # --- Gripper Setup ---
        self.grip_client = ActionClient(self, Grip, '/griplink_node/grip')
        self.release_client = ActionClient(self, Release, '/griplink_node/release')
        self.gripper_port = 0
        self.gripper_index = 1  # Dein Preset
        
        # Hilfsvariablen um Mehrfach-Triggerung bei einem Tastendruck zu vermeiden
        self.last_buttons = [0] * 12 

        # Einstellungen
        self.max_speed = 150.0
        self.max_rot_speed = 30.0
        self.deadzone = 0.05
        
        self.target_vel = [0.0, 0.0, 0.0]
        self.target_rot = [0.0, 0.0, 0.0]
        
        self.timer = self.create_timer(0.02, self.publish_jog)
        
        self.get_logger().info("=== PS4 Steuerung mit Greifer bereit ===")
        self.get_logger().info("L-Stick: Vor/Zurück & Seitwärts")
        self.get_logger().info("R2/L2: Hoch/Runter (Z)")
        self.get_logger().info("R-Stick: Pitch & Yaw")
        self.get_logger().info("L1/R1: Roll")
        self.get_logger().info("X: Greifen | Kreis: Öffnen")

    def filter_stick(self, value):
        return value if abs(value) > self.deadzone else 0.0

    # --- Greifer Funktionen (Asynchron) ---
    def call_grip(self):
        if self.grip_client.wait_for_server(timeout_sec=1.0):
            goal = Grip.Goal(port=self.gripper_port, index=self.gripper_index)
            self.grip_client.send_goal_async(goal)

    def call_release(self):
        if self.release_client.wait_for_server(timeout_sec=1.0):
            goal = Release.Goal(port=self.gripper_port, index=self.gripper_index)
            self.release_client.send_goal_async(goal)

    def joy_callback(self, msg):
        # 1. BEWEGUNG (Mapping wie besprochen)
        self.target_vel[0] = self.filter_stick(msg.axes[1]) * self.max_speed   # Vor/Zurück
        self.target_vel[1] = self.filter_stick(msg.axes[0]) * self.max_speed   # Seitlich
        
        r2 = (msg.axes[5] - 1.0) / -2.0 if msg.axes[5] != 0.0 else 0.0
        l2 = (msg.axes[2] - 1.0) / -2.0 if msg.axes[2] != 0.0 else 0.0
        self.target_vel[2] = (r2 - l2) * self.max_speed                       # Z-Achse

        self.target_rot[1] = self.filter_stick(msg.axes[4]) * self.max_rot_speed # Pitch
        self.target_rot[2] = self.filter_stick(msg.axes[3]) * self.max_rot_speed # Yaw
        
        roll_dir = float(msg.buttons[5] - msg.buttons[4])
        self.target_rot[0] = roll_dir * self.max_rot_speed                    # Roll

        # 2. GREIFER-STEUERUNG (Buttons)
        # msg.buttons[0] ist meist 'X', msg.buttons[1] ist 'Kreis' oder 'O'
        # Wir prüfen, ob die Taste JETZT gedrückt ist, aber im LETZTEN Frame nicht (Edge Detection)
        
        if msg.buttons[0] == 1 and self.last_buttons[0] == 0:
            self.call_grip()
        
        if msg.buttons[1] == 1 and self.last_buttons[1] == 0:
            self.call_release()

        self.last_buttons = msg.buttons

    def publish_jog(self):
        msg = JogLinear()
        msg.vel = [float(x) for x in self.target_vel]
        msg.rot = [float(x) for x in self.target_rot]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JogLinearJoyControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop-Befehl senden
        stop = JogLinear(vel=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0])
        node.publisher_.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
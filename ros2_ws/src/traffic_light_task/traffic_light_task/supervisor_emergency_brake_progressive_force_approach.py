import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

REF_DISTANCE = 3.0
TL_SEEN_DELAY_CONST = 6 # 6x2ms = 12ms antes de interpretrar que ya no hay semáforo
BREAK_DISTANCE = 15.0
P_CONTROL = 1.0

class EmergencyBreakSupervisor(Node):
    def __init__(self):
        super().__init__('supervisor_eb')

        timer_period = 0.02  # 2 mseconds period
        self.timer = self.create_timer(timer_period, self.run)

        # Subscriptors
        self.subscription_control_cmd = self.create_subscription(
            Twist,
            '/intellicar/ego_vehicle/control_command',
            self.control_cmd_callback,
            10)
        
        self.subscription_tl_type = self.create_subscription(
            String,
            '/intellicar/ego_vehicle/traffic_light_type',
            self.tl_type_callback,
            10)

        self.subscription_est_distance = self.create_subscription(
            Float32,
            '/intellicar/ego_vehicle/traffic_light/dist_est',
            self.estimated_distance_callback,
            10)
        
        self.subscription_speed = self.create_subscription(
            Float32,
            '/carla/ego_vehicle/speedometer',
            self.car_speed_callback,
            10)
        
        # Publishers
        self.publisher_motion_control = self.create_publisher(Twist, '/carla/ego_vehicle/twist', 10)

        # Params
        self.car_speed = 0.0
        self.tf_light_type = "No"
        self.manual_control_command = Twist()
        self.distance_to_target = 50.0
        self.tl_seen_delay = 0
        self.new_tl_detection = False

    def car_speed_callback(self, msg):
        self.car_speed = msg.data

    def tl_type_callback(self, msg):
        self.tf_light_type = msg.data
        self.new_tl_detection = True
    
    def estimated_distance_callback(self, msg):
        self.distance_to_target = msg.data

    def control_cmd_callback(self, msg):
        self.manual_control_command = msg

    def run(self):
        self.supervised_cmd = Twist()

        # Check for recent semaphore detection
        if not self.new_tl_detection:
            self.tl_seen_delay = self.tl_seen_delay + 1
        else:
            self.new_tl_detection = False
            self.tl_seen_delay = 0
            self.tl_seen = True
        if self.tl_seen_delay > TL_SEEN_DELAY_CONST: # ya no se ve semaforo si durante 12ms no se recibe callback de detección de este
            self.tl_seen = False

        red_semaphore_condition = (self.tf_light_type == 'Red' and self.tl_seen)
        semaphore_near_condition = (self.distance_to_target < BREAK_DISTANCE)
        low_approach_detected = (self.car_speed < 2.0 and self.distance_to_target > REF_DISTANCE)
        no_brakes_applied_by_user_condition = (self.manual_control_command.linear.x >= 0)
        ref_passed_condition = (self.distance_to_target < REF_DISTANCE)

        if red_semaphore_condition and semaphore_near_condition and \
                (not low_approach_detected) and no_brakes_applied_by_user_condition \
                and not ref_passed_condition:
            self.supervised_cmd.linear.x = P_CONTROL*((10/BREAK_DISTANCE-REF_DISTANCE)*(self.distance_to_target-REF_DISTANCE)-10)
            self.supervised_cmd.angular.z = 0.0
            print('FRENADA AUTÓNOMA DE EMERGENCIA, fuerza: ',self.supervised_cmd.linear.x)
        else:
            self.supervised_cmd = self.manual_control_command
        self.publisher_motion_control.publish(self.supervised_cmd)
        
def main(args=None):
    rclpy.init(args=args)
    ebSupervisor = EmergencyBreakSupervisor()

    rclpy.spin(ebSupervisor)
    ebSupervisor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

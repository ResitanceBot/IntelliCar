import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

EMERGENCY_DISTANCE = 10.0
REF_DISTANCE = 1.0
P_CONTROL = 1.0

class EmergencyBreakSupervisor(Node):
    def __init__(self):
        super().__init__('supervisor_eb')

        timer_period = 0.02  # seconds
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


    def car_speed_callback(self, msg):
        self.car_speed = msg.data

    def tl_type_callback(self, msg):
        self.tf_light_type = msg.data
        self.tl_seen = True
    
    def estimated_distance_callback(self, msg):
        self.distance_to_target = msg.data

    def control_cmd_callback(self, msg):
        self.manual_control_command = msg

    def run(self):
        self.supervised_cmd = Twist()
        # Emergency brake condition: tf_light RED , Distance to tf_light, Positive speed, No manual brake.
        print(self.distance_to_target < EMERGENCY_DISTANCE)
        print(self.tf_light_type)
        print(self.tf_light_type == 'Red')
        # if self.tf_light_type == 'Red' and self.distance_to_target < EMERGENCY_DISTANCE*self.car_speed \
        #         and self.car_speed > 0.01 and self.manual_control_command.linear.x >= 0:
        #     self.supervised_cmd.linear.x = P_CONTROL*((10/EMERGENCY_DISTANCE-REF_DISTANCE)*(self.distance_to_target-REF_DISTANCE)-10)*self.car_speed
        #     self.supervised_cmd.angular.z = 0.0
        if self.tf_light_type == 'Red' and self.distance_to_target < EMERGENCY_DISTANCE \
                and self.car_speed > 2 and self.manual_control_command.linear.x >= 0:
            self.supervised_cmd.linear.x = P_CONTROL*((10/EMERGENCY_DISTANCE-REF_DISTANCE)*(self.distance_to_target-REF_DISTANCE)-10)
            self.supervised_cmd.angular.z = 0.0
            # self.tl_seen = False
            print('ya freno yo makina')
        else:
            print('todo tuyo jfe')
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

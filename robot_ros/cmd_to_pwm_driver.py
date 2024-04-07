import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from time import sleep
from geometry_msgs.msg import Twist #vedi con 'ros2 interface show geometry_msgs/msg/Twist'


class VelocitySubscriber(Node):
    '''
    Implementa l'hardware interface in un nodo ros
    Utile per utilizzare un bridge L298N con Raspberry
    Riceve la velocità sotto forma di msg Twist dai topic cmd_vel
    La converte in impulsi PWM per il motor driver 
    '''

    def __init__(self):
        super().__init__('velocity_subscriber')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_to_pwm_callback, 10)
        # self.subscription  # prevent unused variable warning
        
        # Pins for left Motor Driver Inputs 
        # Nota: dichiaro self. solo quelli che devono essere 
        # accessibili fuori dalla classe
        Motor1A = 24
        Motor1B = 23
        Motor1E = 25 #enable
        # Pins for right Motor Driver Inputs
        Motor2A = 14
        Motor2B = 13
        Motor2E = 15 #enable

        #setup 
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)        # GPIO Numbering
        GPIO.setup(Motor1A,GPIO.OUT)  # All pins as Outputs
        GPIO.setup(Motor1B,GPIO.OUT)
        GPIO.setup(Motor1E,GPIO.OUT)

        GPIO.setup(Motor2A,GPIO.OUT)  # All pins as Outputs
        GPIO.setup(Motor2B,GPIO.OUT)
        GPIO.setup(Motor2E,GPIO.OUT)    

        pwm_l = GPIO.PWM(Motor1E, 1000) #1000 è lo step di incremento del PWM
        pwm_r = GPIO.PWM(Motor2E, 1000)

        pwm_l.start(75)
        pwm_r.start(75)

        self.ml_a = Motor1A
        self.ml_b = Motor1B
        self.mr_a = Motor2A
        self.mr_b = Motor2B
        self.ml_e = Motor1E
        self.mr_e = Motor2E

    def cmd_to_pwm_callback(self, msg):
        # self.get_logger().info('Subscribing "%d"' % msg.data)
        right_wheel_vel = (msg.linear.x + msg.angular.z)/2
        left_wheel_vel = (msg.linear.x - msg.angular.z)/2
        print(right_wheel_vel, " / ", left_wheel_vel)

        GPIO.output(self.ml_a, left_wheel_vel > 0)
        GPIO.output(self.ml_b, left_wheel_vel < 0)
        GPIO.output(self.ml_e, GPIO.HIGH)
        GPIO.output(self.mr_a, right_wheel_vel > 0)
        GPIO.output(self.mr_b, right_wheel_vel < 0)
        GPIO.output(self.mr_e, GPIO.HIGH)


def main(args=None):
    rclpy.init(args=args)

    velocity_subscriber = VelocitySubscriber()

    rclpy.spin(velocity_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    velocity_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
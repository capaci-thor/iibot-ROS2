from cmath import sqrt
from telnetlib import WILL
import rclpy
#Se importa Node porque se hara uso de el
from rclpy.node import Node
#Se importa el tipo de mensaje que el nodo usara para pasar datos sobre el topic
from std_msgs.msg import Int32MultiArray
#biblioteca para el control de las ruedas
import smbus
import time
import math
#import YB_Pcb_Car
#Las lineas pasadas representan las dependencias del nodo que deben ir en 
# package.xml

#car = YB_Pcb_Car.YB_Pcb_Car()

rpm_l = 0.0
rpm_r = 0.0
v_l = 0.0
v_r = 0.0
inter_l = 0
inter_r = 0

class MoveSubscriber(Node):

    def __init__(self):
        
        super().__init__('Lyapunov') #nombre del nodo
        #Mismos parametros que publisher
        self.count = self.create_subscription(Int32MultiArray, 'count', self.listener_callback, 2)
        self.count  # prevent unused variable warning


    def listener_callback(self, msg):
        c_r = msg.data[0]
        c_l = msg.data[1]

        global inter_r
        global rpm_r
        global v_r

        global inter_l
        global rpm_l
        global v_l

        inter_r += c_r
        rpm_r = 60 * (c_r/20)
        v_r = (math.pi * 6.6 * rpm_r)/(100*60) #m/s

        inter_l += c_l
        rpm_l = 60 * (c_l/20)
        v_l = (math.pi * 6.6 * rpm_l)/(100*60) #m/s


        self.get_logger().info('vel left: "%s"' % str(v_l))
        self.get_logger().info('vel right: "%s"' % str(v_r))
        #self.lyapunov()

    def lyapunov(self):
        r = 6.6
        b = 10
        #Gains
        k1 = 1
        k2 = 1
        q2 = 0.5
        #ref en metros
        Pxd = -1
        Pyd = 3.5
        phid = 180 * (math.pi/180)
        #kinematics
        wl = (2*math.pi*rpm_l)/(60)
        wr = (2*math.pi*rpm_r)/(60)
        v = (r*(wl+wr))/2
        w = (r*(wr-wl))/2*b
        self.get_logger().info('vel: "%s"' % str(v))
        self.get_logger().info('w: "%s"' % str(w))
        x = v*(math.cos(w))
        y = v*(math.sin(w))
        




def main(args=None):
    
    rclpy.init(args=args)

    move_subscriber = MoveSubscriber()

    rclpy.spin(move_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    move_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




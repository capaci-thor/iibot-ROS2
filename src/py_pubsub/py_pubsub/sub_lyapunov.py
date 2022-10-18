import rclpy
#Se importa Node porque se hara uso de el
from rclpy.node import Node
#Se importa el tipo de mensaje que el nodo usara para pasar datos sobre el topic
from std_msgs.msg import Int32
#biblioteca para el control de las ruedas
import smbus
import time
import math
import YB_Pcb_Car
#Las lineas pasadas representan las dependencias del nodo que deben ir en 
# package.xml

car = YB_Pcb_Car.YB_Pcb_Car()

class MoveSubscriber(Node):

    def __init__(self):
        
        super().__init__('Lyapunov') #nombre del nodo
        #Mismos parametros que publisher
        self.l_count = self.create_subscription(Int32, 'l_count', self.lc_callback, 1)
        self.r_count = self.create_subscription(Int32, 'r_count', self.lr_callback, 1)
        self.l_count  # prevent unused variable warning
        self.r_count

    def lc_callback(self, msg):
        x = msg.data 
        self.get_logger().info('I heard: "%s"' % str(msg.data))
        #self.get_logger().info('X : "%d"' % x)
        #self.get_logger().info('Y : "%d"' % y)

    def lr_callback(self, msg):
        x = msg.data
        self.get_logger().info('I heard: "%s"' % str(msg.data))
        
        #self.get_logger().info('X : "%d"' % x)
        #self.get_logger().info('Y : "%d"' % y)


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




# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
#Se importa Node porque se hara uso de el
from rclpy.node import Node
#Se importa el tipo de mensaje que el nodo usara para pasar datos sobre el topic
from std_msgs.msg import Int32MultiArray
#biblioteca para el control de las ruedas
import YB_Pcb_Car
#Las lineas pasadas representan las dependencias del nodo que deben ir en 
# package.xml
car = YB_Pcb_Car.YB_Pcb_Car()

class MinimalSubscriber(Node):

    def __init__(self):
        
        super().__init__('minimal_subscriber') #nombre del nodo
        #Mismos parametros que publisher
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'topic',
            self.listener_callback,
            2)
        
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        x = msg.data[0]
        y = msg.data[1]
        self.get_logger().info('I heard: "%s"' % str(msg.data))
        car.Car_Run(x,y)
        self.get_logger().info('X : "%d"' % x)
        self.get_logger().info('Y : "%d"' % y)


def main(args=None):
    
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
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
#Las lineas pasadas representan las dependencias del nodo que deben ir en 
# package.xml

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher') #nombre al nodo
        #El nodo publica mensajes del tipo string en "topic" con tamaño 10
        self.publisher_ = self.create_publisher(Int32MultiArray, 'dir', 2)
        #El temporizador se crea con un callback para ejecutarse cada 
        # 0,5 segundos.
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #es un contador utilizado en el callback
        #self.i = 0
        while True:
            msg = Int32MultiArray()
            cor = str(input("enter data: "))
            corSplit = cor.split(",")
            x = int(corSplit[0])
            y = int(corSplit[1])
            vector = []
            vector.append(x)
            vector.append(y)
            msg.data = vector
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % vector)
        



def main(args=None):

    #Primero se inicializa la biblioteca rclpy, 
    #luego se crea el nodo y luego "spins" el nodo para que 
    #llamen sus callbacks.
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

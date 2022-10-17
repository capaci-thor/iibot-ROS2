import rclpy
#Se importa Node porque se hara uso de el
from rclpy.node import Node
#Se importa el tipo de mensaje que el nodo usara para pasar datos sobre el topic
from std_msgs.msg import Int32
#Las lineas pasadas representan las dependencias del nodo que deben ir en 
# package.xml
from time import sleep

from serial import *


class ConterLeftPublisher(Node):
    ser = Serial()

    def __init__(self):
        super().__init__('publisher_right_encoder') #nombre al nodo
        #El nodo publica mensajes del tipo Float en array en "accel" con tamaño 3
        self.publisher_ = self.create_publisher(Int32, 'r_count',1)
        #El temporizador se crea con un callback para ejecutarse cada 
        # 0,5 segundos.
        ser = Serial(
        port='/dev/ttyAMA0',
        baudrate = 9600,
        parity=PARITY_NONE,
        stopbits=STOPBITS_ONE,
        bytesize=EIGHTBITS,
        timeout=10
        )
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #es un contador utilizado en el callback
        #self.i = 0

    def timer_callback(self):
        msg = Int32() 
        #data = -1
        try:
            ser.write("0".encode())
            x=ser.readline().decode()
            x = x.replace("\n","")
            x = x.replace("\r","")
            data = int(x)
        except:
            print("Serial error")
            sleep(0.5)

        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        #self.i += 1.0


def ConvertStringsToBytes(src):
    converted = []
    for b in src:
        converted.append(ord(b))
    return converted


def main(args=None):

    #Primero se inicializa la biblioteca rclpy, 
    #luego se crea el nodo y luego "spins" el nodo para que 
    #llamen sus callbacks.
    rclpy.init(args=args)

    l_count_publisher = ConterLeftPublisher()

    rclpy.spin(l_count_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    l_count_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
#Se importa Node porque se hara uso de el
from rclpy.node import Node
#Se importa el tipo de mensaje que el nodo usara para pasar datos sobre el topic
from std_msgs.msg import Int32MultiArray
#Las lineas pasadas representan las dependencias del nodo que deben ir en 
# package.xml
from serial import *
import smbus
from time import sleep

ser = Serial(
        port='/dev/ttyAMA0',
        baudrate = 9600,
        parity=PARITY_NONE,
        stopbits=STOPBITS_ONE,
        bytesize=EIGHTBITS,
        timeout=10
        )

I2C_SLAVE_ADDRESS = 0x8 #Arduino was configured for this adress


class DirectionPublisher(Node):

    def __init__(self):
        super().__init__('encoders_publisher') #nombre al nodo
        #El nodo publica mensajes del tipo string en "topic" con tamaño 10
        self.publisher_ = self.create_publisher(Int32MultiArray, 'count', 2)
        #El temporizador se crea con un callback para ejecutarse cada 
        # 0,5 segundos.
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #es un contador utilizado en el callback
        #self.i = 0

    def timer_callback(self):
        global ser
        msg = Int32MultiArray()
        vector = []


        #code for right encoder
        ser.write("0".encode())
        c_r = ser.readline().decode()
        c_r = c_r.replace("\n","")
        c_r = c_r.replace("\r","")
        vector.append(int(c_r))


        #code for left encoder
        bus = smbus.SMBus(1)
        slaveAddress = I2C_SLAVE_ADDRESS

        BytesToSend = ConvertStringsToBytes("1")
        bus.write_byte(slaveAddress,  1)
        try:
            bus.write_byte(slaveAddress,  1)
            c_l = bus.read_byte(slaveAddress)
            c_l = int(c_l)            
        except:
            print("remote i/o error")
            sleep(0.5)
        vector.append(c_l)

        #send msg
        msg.data = vector
        self.publisher_.publish(msg)
        self.get_logger().info('Encoder R L "%s"' % vector)

    
    
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

    direction_publisher = DirectionPublisher()

    rclpy.spin(direction_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    direction_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
#Se importa Node porque se hara uso de el
from rclpy.node import Node
#Se importa el tipo de mensaje que el nodo usara para pasar datos sobre el topic
from std_msgs.msg import Int32
#Las lineas pasadas representan las dependencias del nodo que deben ir en 
# package.xml
#For I2C com
import smbus
from time import sleep


I2C_SLAVE_ADDRESS = 0x8 #Arduino was configured for this adress

class ConterLeftPublisher(Node):

    def __init__(self):
        super().__init__('publisher_accelerometer') #nombre al nodo
        #El nodo publica mensajes del tipo Float en array en "accel" con tamaño 3
        self.publisher_ = self.create_publisher(Int32, 'l_count',1)
        #El temporizador se crea con un callback para ejecutarse cada 
        # 0,5 segundos.
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #es un contador utilizado en el callback
        self.i = 0

    def timer_callback(self):
        msg = Int32()    
        bus = smbus.SMBus(1)
        slaveAddress = I2C_SLAVE_ADDRESS

        BytesToSend = self.ConvertStringsToBytes("1")
        bus.write_byte(slaveAddress,  1)
        sleep(0.1)

        try:
            bus.write_byte(slaveAddress,  1)
            data=bus.read_byte(slaveAddress)
            #print(data)
            data = int(data)
        except:
            print("remote i/o error")
            sleep(0.5)
        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1.0

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

import rclpy
#Se importa Node porque se hara uso de el
from rclpy.node import Node
#Se importa el tipo de mensaje que el nodo usara para pasar datos sobre el topic
from std_msgs.msg import Float32MultiArray
#Las lineas pasadas representan las dependencias del nodo que deben ir en 
# package.xml
#For I2C com
import smbus
from time import sleep

PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38

ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F

Device_Address = 0x68   # MPU6050 device address
bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
sleep(1)

def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

class AccelPublisher(Node):

    def __init__(self):
        super().__init__('publisher_accelerometer') #nombre al nodo
        #El nodo publica mensajes del tipo Float en array en "accel" con tamaño 3
        self.publisher_ = self.create_publisher(Float32MultiArray, 'accel', 3)
        #El temporizador se crea con un callback para ejecutarse cada 
        # 0,5 segundos.
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #es un contador utilizado en el callback
        self.i = 0

    def timer_callback(self):
        msg = Float32MultiArray()    
        MPU_Init()
        acc_x = read_raw_data(ACCEL_XOUT_H)/16384.0
        acc_y = read_raw_data(ACCEL_YOUT_H)/16384.0
        acc_z = read_raw_data(ACCEL_ZOUT_H)/16384.0
        acc = []
        acc.append(acc_x)
        acc.append(acc_y)
        acc.append(acc_z)
        msg.data = acc
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1.0
        



def main(args=None):

    #Primero se inicializa la biblioteca rclpy, 
    #luego se crea el nodo y luego "spins" el nodo para que 
    #llamen sus callbacks.
    rclpy.init(args=args)

    accel_publisher = AccelPublisher()

    rclpy.spin(accel_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    accel_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

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

############################################################
#objeto motores
class YB_Pcb_Car(object):

    def get_i2c_device(self, address, i2c_bus):
        self._addr = address
        if i2c_bus is None:
            return smbus.SMBus(1)
        else:
            return smbus.SMBus(i2c_bus)

    def __init__(self):
        # Create I2C device.
        self._device = self.get_i2c_device(0x16, 1)

    def write_u8(self, reg, data):
        try:
            self._device.write_byte_data(self._addr, reg, data)
        except:
            print ('write_u8 I2C error')

    def write_reg(self, reg):
        try:
            self._device.write_byte(self._addr, reg)
        except:
            print ('write_u8 I2C error')

    def write_array(self, reg, data):
        try:
            # self._device.write_block_data(self._addr, reg, data)
            self._device.write_i2c_block_data(self._addr, reg, data)
        except:
            print ('write_array I2C error')

    def Ctrl_Car(self, l_dir, l_speed, r_dir, r_speed):
        try:
            reg = 0x01
            data = [l_dir, l_speed, r_dir, r_speed]
            self.write_array(reg, data)
        except:
            print ('Ctrl_Car I2C error')
            
    def Control_Car(self, speed1, speed2):
        try:
            if speed1 < 0:
                dir1 = 0
            else:
                dir1 = 1
            if speed2 < 0:
                dir2 = 0
            else:
                dir2 = 1 
            
            self.Ctrl_Car(dir1, int(math.fabs(speed1)), dir2, int(math.fabs(speed2)))
        except:
            print ('Ctrl_Car I2C error')


    def Car_Run(self, speed1, speed2):
        try:
            self.Ctrl_Car(1, speed1, 1, speed2)
        except:
            print ('Car_Run I2C error')

    def Car_Stop(self):
        try:
            reg = 0x02
            self.write_u8(reg, 0x00)
        except:
            print ('Car_Stop I2C error')

    def Car_Back(self, speed1, speed2):
        try:
            self.Ctrl_Car(0, speed1, 0, speed2)
        except:
            print ('Car_Back I2C error')

    def Car_Left(self, speed1, speed2):
        try:
            self.Ctrl_Car(0, speed1, 1, speed2)
        except:
            print ('Car_Spin_Left I2C error')

    def Car_Right(self, speed1, speed2):
        try:
            self.Ctrl_Car(1, speed1, 0, speed2)
        except:
            print ('Car_Spin_Left I2C error')

    def Car_Spin_Left(self, speed1, speed2):
        try:
            self.Ctrl_Car(0, speed1, 1, speed2)
        except:
            print ('Car_Spin_Left I2C error')

    def Car_Spin_Right(self, speed1, speed2):
        try:
            self.Ctrl_Car(1, speed1, 0, speed2)
        except:
            print ('Car_Spin_Right I2C error')

    def Ctrl_Servo(self, id, angle):
        try:
            reg = 0x03
            data = [id, angle]
            if angle < 0:
                angle = 0
            elif angle > 180:
                angle = 180
            self.write_array(reg, data)
        except:
            print ('Ctrl_Servo I2C error') 

############################################################
car = YB_Pcb_Car()

rpm_l = 0.0
rpm_r = 0.0

v_l = 0.0
v_r = 0.0

inter_l = 0
inter_r = 0

#Lyapunov
#Condiciones iniciales
#Creación de matrices
#x = []
#y = []
#phi = []
#Verdaderas condciones iniciales
#x.append(0)
#y.append(0)
#phi.append(0)

#Errores
#iota = []
#dseta = []
#psi = []

#Control
#uref = []
#wref = []

class MoveSubscriber(Node):

    def __init__(self):
        
        super().__init__('Lyapunov') #nombre del nodo
        #Mismos parametros que publisher
        self.count = self.create_subscription(Int32MultiArray, 'count', self.listener_callback, 2)
        self.i = 0
        self.count  # prevent unused variable warning
        #Lyapunov
        #Condiciones iniciales
        #Creación de matrices
        self.x = []
        self.y = []
        self.phi = []
        #Verdaderas condciones iniciales
        self.x.append(0)
        self.y.append(0)
        self.phi.append(0)

        #Errores
        self.iota = []
        self.dseta = []
        self.psi = []

        #Control
        self.uref = []
        self.wref = []


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
        self.lyapunov()

    def lyapunov(self):
        
        r = 6.6
        b = 10
        #Gains
        k1 = 1
        k2 = 1
        q2 = 0.5
        #ref en metros
        Pxd = 1
        Pyd = 3.5
        phid = 0 * (math.pi/180)
        #kinematics
        wl = (2*math.pi*rpm_l)/(60)
        wr = (2*math.pi*rpm_r)/(60)
        v = (r*(wl+wr))/2
        w = (r*(wr-wl))/2*b
        self.get_logger().info('vel: "%s"' % str(v))
        self.get_logger().info('w: "%s"' % str(w))
  
        self.iota.append( math.sqrt(((Pxd - self.x[self.i])**2) + ((Pyd - self.y[self.i])**2)) )
        self.dseta.append( math.atan2((Pyd - self.y[self.i]),(Pxd - self.x[self.i]) - self.phi[self.i]))
        self.psi.append( math.atan2((Pyd - self.y[self.i]),(Pxd - self.x[self.i])-phid) )

        #control

        self.uref.append( k1*math.cos(self.dseta[self.i] * self.iota[self.i]) )
        self.wref.append( k2*self.dseta[self.i] + (k1/self.dseta[self.i]) * math.cos(self.dseta[self.i]) * math.sin(self.dseta[self.i]) * (self.dseta[self.i] + q2 * self.psi[self.i]) )

        #rb

        robot(self.uref[self.i] , self.wref[self.i])
        xp = v * math.cos(self.phi[self.i])
        yp = v * math.sin(self.phi[self.i])
        self.phi.append( w + self.phi[self.i]) 

        self.x.append( xp + self.x[self.i] )
        self.y.append( yp + self.y[self.i] )
        self.i+=1

def robot(v, w):
    

    if w > 0:
        l = int(w*(152.98*v) + 4.0434)
        r = int((164.24*v) - 1.3834)
        ml = l if l <= 255 else 255
        mr = r if r <= 255 else 255
        car.Car_Run(ml,mr)

    elif w < 0:
        l = int((152.98*v) + 4.0434)
        r = int(w*(164.24*v) - 1.3834)
        ml = l if l <= 255 else 255
        mr = r if r <= 255 else 255
        car.Car_Run(ml,mr)




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




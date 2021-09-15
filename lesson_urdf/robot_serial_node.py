# Copyright 2021 Utadeo
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
from rclpy.node import Node
import time
import serial

from sensor_msgs.msg import JointState

from std_msgs.msg import Header
from math import pi


class RobotSerialNode(Node):

    def __init__(self):
        super().__init__('serial_robot_node')
        self.joint_state_sub_ = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            1)

        try:
            self.serial= serial.Serial(timeout=0.045,write_timeout=0.001)
            self.serial.baudrate = 115200
            self.serial.port = '/dev/ttyACM0'
            self.serial.open()
            time.sleep(1)
        except:
            self.get_logger().warning('Error al abrir el puerto Serial, saliendo')
            #quit()

        #TODO: adicionar parametros (configurar correctamente)
        self.t_s = 0.05  # seconds
        self.incremento = 1 # incremento por letra
        #self.timer = self.create_timer(self.t_s, self.timer_callback)
        self.time_1=self.get_clock().now()
        self.q=[0,0,0,0,0,0]
        self.q_1=self.q
        #self.serial.write(b'h')

    def move_serial(self,q_move):
        '''
        the protocol is one letter for each degree of movement of robot
        example first motor use a(-1) or A(+1)
        
         chr(ord(a)+1)
        '''
        comandos=''
        for move,i in zip(q_move,range(0, len(q_move))):
            command=65
            if move>0:
                for j in range(0,int(move)):
                    #self.serial.write(char(command+i))
                    comandos+=chr(command+i)
            elif move<0:
                for j in range(int(move),0):
                    #self.serial.write(b'S')
                    comandos+=chr(command+i+32)
        self.serial.write(comandos.encode())

    def joint_state_callback(self,msg):
        """
        Esta funciòn recibe el tópico para comandar el robot
        """
        #print(list(msg.position))
        self.q=list(msg.position)
        
        if sum(self.q)!=sum(self.q_1):
            move=self.q.copy()
            suma=0
            for i in range(0,len(move)):
                move[i]=round((self.q[i]-self.q_1[i])*180.0/pi)
                suma += abs(move[i])
                self.q_1[i]=self.q[i]
            self.get_logger().info('moving: "%s"' % move)
            if(suma>0):
                    None#self.move_serial(move)
            


        

    def clamp(self,num, min_value, max_value):
        """
        Función para limitar el valor de las señales
        """
        return max(min(num, max_value), min_value)
    def timer_callback(self):
        """
        Esta función es periodica y es la encargada de enviar el comando y recibir la información del serial
        """
        
        None

  


def main(args=None):
    rclpy.init(args=args)

    robot_serial_node = RobotSerialNode()

    rclpy.spin(robot_serial_node)

    # Destroy the node explicitly (optional)
    robot_serial_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
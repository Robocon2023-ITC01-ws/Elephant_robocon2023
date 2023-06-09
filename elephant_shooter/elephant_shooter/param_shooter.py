import yaml
import rclpy
import numpy as np
from rclpy.node import Node
import getpass
username = getpass.getuser()

from std_msgs.msg import Float32MultiArray

def write_yaml_to_file(py_obj,filename):
    with open(f'{filename}', 'w',) as f :
        yaml.dump(py_obj,f,sort_keys=True) 
    print('Written to file successfully')

def read_and_modify_one_block_of_yaml_data(filename,write_file, key,value):
    with open(f'{filename}', 'r') as f:
        data = yaml.safe_load(f)
        data[f'{key}'] = value
        print(data)
    with open(f'{write_file}', 'w') as file:
        yaml.dump(data,file,sort_keys=True)
    print('done!') 

class YamlNode(Node):
    def __init__(self):
        super().__init__('yaml_node')
        self.param_sub = self.create_subscription(Float32MultiArray, 'stored', self.param_callback, 100)
        self.data = {'distance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0]}
        self.distance = np.zeros(25)
        self.file = f'/home/{username}/elephant_ws/src/elephant_shooter/config/param.yaml'

    def param_callback(self, param_msg):
        self.distance = param_msg.data
        read_and_modify_one_block_of_yaml_data(self.file, self.file, key='distance', value=[self.distance[0],self.distance[1],self.distance[2], self.distance[3], self.distance[4],
                                                                                                  self.distance[5],self.distance[6],self.distance[7], self.distance[8], self.distance[9],
                                                                                                  self.distance[10],self.distance[11],self.distance[12], self.distance[13], self.distance[14],
                                                                                                  self.distance[15],self.distance[16],self.distance[17], self.distance[18], self.distance[19],
                                                                                                  self.distance[20],self.distance[21],self.distance[22], self.distance[23], self.distance[24]])

def main(args=None):
    rclpy.init(args=args)
    yaml_node = YamlNode()
    rclpy.spin(yaml_node)
    yaml_node.destroy_node()
    rclpy.shutdown

if __name__=='__main__':
    main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from zed_interfaces.msg import ObjectsStamped
import math
class ZedConversion(Node):
    def __init__(self):
        super().__init__('zed_conversion')
        self.zed_objects_subscriber = self.create_subscription(ObjectsStamped, '/real/zed/obj_det/objects', self.zed_objects_callback, 10)
        self.pose_publisher = self.create_publisher(String, '/real/obj_det_pose', 10)
        
        self.OKGREEN = '\033[92m'
        self.ENDC = '\033[0m'
        
        self.pose = String()
        
        self.get_logger().info(self.OKGREEN + "Configured and Activated Zed Conversion" + self.ENDC)
        
    def zed_objects_callback(self, objects: ObjectsStamped):
        if objects == None:
            self.get_logger().warn("Objects message recieved was null")
        else:
            if len(objects.objects) <= 0:
                self.get_logger().warn("NO OBJECTS DETECTED")
                empty = String()
                empty.data = "0.0|0.0|0.0"
                self.pose_publisher.publish(empty)
            else:
                best_notes = []
                for i in objects.objects:
                    if i.confidence > 0.5:
                        best_notes.append(i)
                best_notes.sort(key = lambda x: math.sqrt(math.pow(x.position[0], 2) + math.pow(x.position[1], 2) + math.pow(x.position[2], 2)))
                x = best_notes[0].position[0]
                y = best_notes[0].position[1]
                z = best_notes[0].position[2]
                self.pose.data = f"{float(x)}|{float(y)}|{float(z)}"
                print(self.pose)
                self.pose_publisher.publish(self.pose)
            
def main(args=None):
    rclpy.init(args=args)
    
    zed_conversion = ZedConversion()
    rclpy.spin(zed_conversion)
    
    zed_conversion.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
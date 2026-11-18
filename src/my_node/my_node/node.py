import copy
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from time import sleep
import panda_py
from std_msgs.msg import String, Float32MultiArray
from std_srvs.srv import Trigger

from grasp_interfaces.msg import TrackedObject, TrackedObjectArray

temp = [[0, 0]]

class StateMachine(Node):
    def __init__(self):
        super().__init__("Controller")
        self.count = 0
        self.ready_send = "ready"

        n = panda_py.Panda('172.16.0.2').get_position()
        print(f'POSITION FROM TOP OF LINK0 TO END EFFECTOR: {n}')

        # My Subscriptions
        self.exgf_sub = self.create_subscription(String, '/state', self.handle_ex ,1)

        # My Publishers
        self.send_instruct = self.create_publisher(String, '/command', 1)
        self.send_coord = self.create_publisher(Float32MultiArray, '/coord', 1)
        self.sub = self.create_subscription(TrackedObjectArray, '/tracked_objects', self.n, 1)
        # Service for Manual Terminal Call
        self.srv = self.create_service(Trigger, 'publish_command', self.handle_trigger)

        # Wait for RVIZ to Open
        sleep(6)

        # Go
        self.coordsend()

    def n(self, msg):
        
        print(msg)

    def handle_ex(self, msg):
        if msg.data == "runnin":
            self.ready_send = "wait"
        elif msg.data == "not runnin":
            self.ready_send = "ready"
            self.coordsend()
        else:
            print("\n not good \n")

    def coordsend(self):
        pass
        # if self.ready_send == "ready":
        #     msg = Float32MultiArray(data=temp[self.count])
        #     self.send_coord.publish(msg)
        #     sleep(1)
        #     self.n()
        #     self.count +=1
        #     if self.count == len(temp):
        #         self.count = 0
        
    def handle_trigger(self, request, response):
        self.coordsend()
        # msg = String()
        # msg.data = "start"
        # self.send_instruct.publish(msg)
        response.success = True
        response.message = "Published"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
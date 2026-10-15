import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Int32
from std_srvs.srv import Trigger
from time import sleep

temp = [[0, 0]]

class Controller(Node):
    def __init__(self):
        super().__init__("Controller")
        self.count = 0
        self.ready_send = "ready"

        # My Subscriptions
        self.exgf_sub = self.create_subscription(String, '/state', self.handle_ex ,1)
        # self.objCoord = self.create_subscription(Float32MultiArray, '/coord', self.coordsend, 1)
        self.objType = self.create_subscription(Int32, '/type', self.dummy, 1)
        # My Publishers
        self.send_instruct = self.create_publisher(String, '/command', 1)
        self.send_coord = self.create_publisher(Float32MultiArray, '/coord', 1)

        # Wait for RVIZ to Open
        sleep(6)


    def n(self):
        msg = String(data='start')
        self.send_instruct.publish(msg)
    def dummy(self, msg):
        print(msg.data)
        pass
    def handle_ex(self, msg):
        if msg.data == "runnin":
            self.ready_send = "wait"
        elif msg.data == "not runnin":
            self.ready_send = "ready"
            # self.coordsend()
        else:
            print("\n not good \n")

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
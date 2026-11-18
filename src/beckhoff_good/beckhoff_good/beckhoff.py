import rclpy
from rclpy.node import Node
from time import sleep
import pyads
from grasp_interfaces.msg import GraspCandidate, TrackedObjectArray, TrackedObject
import numpy as np
from rclpy.time import Time
from std_msgs.msg import String
# Configuration parameters

LINUX_IP = "169.254.100.100" # ipv4 of ethernet port that connects to PLC
                             # to display ip addresses: linux >> bash >> ip -brief address show
LINUX_NETID = "169.254.100.100.1.1" # add .1.1 to LINUX_IP

PLC_IP = "169.254.184.218" # windows >> twincat ide >> ...
PLC_NETID = "5.155.180.125.1.1" # windows >> twincat ide >> ...
speed = 0.0785398163397

class Gopal(Node):
    def __init__(self):
        super().__init__('Gopal')
        self._logger.warn('\033[1mIs HDD LED green? Was Gantry Homed?\033[0m')
        
        # Subscription
        self.sub = self.create_subscription(TrackedObject, '/gantry/target', self.callback, 1)

        # 
        self.obj_x = None
        self.obj_y = None
        self.obj_stamp = None   # rclpy Time

        # Publisher
        self.send_state = self.create_publisher(String, '/gantry_state', 1)
        
        # PYADS
        pyads.open_port()
        try:
            # check in windows >> twincat >> static routes
            # if a route has not been added:
            # run this code once, then comment this

            # route = pyads.add_route_to_plc(
            #     sending_net_id=LINUX_NETID,
            #     adding_host_name=LINUX_IP,
            #     ip_address=PLC_IP,
            #     username="Administrator",
            #     password="1",
            #     route_name="Linux")

            self.plc = pyads.Connection(PLC_NETID, pyads.PORT_TC3PLC1, PLC_IP)
            self.plc.open()

            self._logger.info('Connection verfied. Now waiting for TrackedObject')
            self.send_state.publish(String(data='not runnin'))

        except Exception as e:
            self._logger.fatal(f"\nException -> {e}")
            pyads.close_port()
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            raise SystemExit
        
    def callback(self, msg):
        self.send_state.publish(String(data='runnin'))
        self._logger.info('TrackedObject Received!')

        Y_DISTANCE_BETWEEN = 1.31 # check again

        # if self.obj_stamp is not None:   # already got one, ignore
        #     return
        dt = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        
        delay = int((Y_DISTANCE_BETWEEN + msg.position.y)/speed - dt) - 3
        x_now = msg.position.x

        # My cool transformation
        x_cmd = 39000 + int(x_now * 100000) # 1000 machine unit is 1 cm # 39000

        print(dt)
        print(delay)
        self.plc.write_by_name("MAIN.rCmdX", x_cmd)
        self.plc.write_by_name("MAIN.nPickDelayMs", delay*1000)
        self.plc.write_by_name("MAIN.nStatus", 3)

        sleep(0.5)

        # david patent wait until executed
        while self.plc.read_by_name("MAIN.nStatus") != 2:
            print(self.plc.read_by_name("MAIN.nStatus"))
            sleep(2)
        print('done????')

        #Step 3. Close
        self.send_state.publish(String(data='not runnin'))
        self._logger.info('Finished Executing... Waiting for Tracked Object')

def main():
    rclpy.init()
    node = Gopal()
    try:
        rclpy.spin(node)
    finally:
        pyads.close_port()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from time import sleep
from std_msgs.msg import Float32MultiArray
import pyads
from grasp_interfaces.msg import GraspCandidate

# Configuration parameters

LINUX_IP = "169.254.100.100" # ipv4 of ethernet port that connects to PLC
                             # to display ip addresses: linux >> bash >> ip -brief address show
LINUX_NETID = "169.254.100.100.1.1" # add .1.1 to LINUX_IP

PLC_IP = "169.254.184.218" # windows >> twincat ide >> ...
PLC_NETID = "5.155.180.125.1.1" # windows >> twincat ide >> ...

# pyads.add_route(PLC_NETID, PLC_IP)
def verify_connect():
    pyads.open_port()

    # i think only do this once; check in windows >> twincat >> static routes
    route = pyads.add_route_to_plc(
        sending_net_id=LINUX_NETID,
        adding_host_name=LINUX_IP,
        ip_address=PLC_IP,
        username="Administrator",
        password="1",
        route_name="Linux"
    )
    plc = pyads.Connection(PLC_NETID, pyads.PORT_TC3PLC1, PLC_IP)

    try:
        plc.open()
        print(f'{plc.read_state()}') # (5, 0) means PLC is in run mode and will listen to pyads
        
    except Exception as e:
        print(f"\nException: {e}")
        print('ERROR IN EXECUTING, CHECK EXCEPTION')
        print('ERROR IN EXECUTING, CHECK EXCEPTION')
        print('ERROR IN EXECUTING, CHECK EXCEPTION')
        print('ERROR IN EXECUTING, CHECK EXCEPTION')
    finally:
        plc.close()

    pyads.close_port()

#verify_connect() #un commnet this when staritng. we want to first verifiy that we have a connection before running the node

class Gopal(Node):
    def __init__(self):
        super().__init__('Gopal')
        print('hi')
        # self.sub = self.create_subscription(Float32MultiArray, '/gantry_object_cloud', self.callback, 1)
        self.sub = self.create_subscription(GraspCandidate, '/grasp_candidates', self.cand_callback, 1)
        self.obj_x = None
        self.obj_y = None
        self.obj_stamp = None   # rclpy Time
        
    def cand_callback(self, msg):
        if self.obj_stamp is not None:   # already got one, ignore
            return

        self.obj_x = msg.pose.position.x
        self.obj_y = msg.pose.position.y
        self.obj_stamp = self.get_clock().now()

        self.get_logger().info(
            f"Saved x={self.obj_x:.4f}, y={self.obj_y:.4f}, "
            f"t={self.obj_stamp.nanoseconds * 1e-9:.3f}s"
        )

        # Stop receiving further messages
        self.destroy_subscription(self.sub)
        self.sub = None

    def callback(self, msg): # YOU HAVE ACCESS TO OBJECT x and Y, solve for how long it will take for object to get there with the time stamp and conveyor speed and thatll tell you when to activate gantry.
        # object_x_position, object_y_position = msg.data[0], msg.data[1]


        # Take time stamp 
        #Step 1.
        # please convert it to whatever the gantry uses

        return # remove this when working on the code
    
        # Step 2. Open connection
        # make it move
        pyads.open_port()
        plc = pyads.Connection(PLC_NETID, pyads.PORT_TC3PLC1, PLC_IP)
        
        try:
            plc.open()
            print(f'{plc.read_state()}') # (5, 0) means PLC is in run mode

            # fill code here 
        
        except Exception as e:
            print(f"\nException: {e}")
            print('ERROR IN EXECUTING, CHECK EXCEPTION')
            print('ERROR IN EXECUTING, CHECK EXCEPTION')
            print('ERROR IN EXECUTING, CHECK EXCEPTION')
            print('ERROR IN EXECUTING, CHECK EXCEPTION')
        finally:
            plc.close()

        #Step 3. Close
        pyads.close_port()
        sleep(20) # keep, don't worry about this

def main():
    rclpy.init()
    node = Gopal()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
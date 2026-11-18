
import rclpy
from rclpy.node import Node

from time import sleep
import math
import copy

from task_planner.conveyor_visualizer import ConveyorVisualizer

from grasp_interfaces.msg import TrackedObject, TrackedObjectArray
from std_msgs.msg import Header, Float32, Float32MultiArray, Int32, String
from rclpy.time import Time
import numpy as np
from dataclasses import dataclass

CONVEYOR_SPEED = 0.0785      # m/s, belt moves toward +y   
CONVEYOR_LENGTH = 195 #cm
CONVEYOR_WIDTH = 60 #cm
NUM_BINS_X = 5
NUM_BINS_Y = 30
#current bins in the y dimension are shifted back by 4
ARM_BIN_RANGE_Y = (2, 15)
ARM_BIN_EXCLUSION = [(1, 10), (1, 11), (1, 12), (1, 13), (1, 14)]
GANTRY_BIN_RANGE_Y = (10, 14)
GANTRY_BIN_EXLUSION = []
NUM_CLASSES = 5

# class id -> robot. Class ids here are placeholders, use your YOLO ids.

#for reference:
# 0 - Glass
# 1 - Metal
# 2 - Paper
# 3 - Cardboard
# 4 - Plastic

TRACKED_TOPIC = '/tracked_objects'
FRANKA_TARGET_TOPIC = '/franka/target'          # placeholder
GANTRY_TARGET_TOPIC = '/gantry/target'          # placeholder
FRANKA_STATE_TOPIC = '/executor_state'        # placeholder
GANTRY_STATE_TOPIC = '/gantry_state' 

@dataclass
class Robot:
    name: str
    reachable_positions: np.ndarray
    busy: int

    @property
    def available(self) -> bool:
        return self.busy

    def act(self) -> None:
        self.busy = 1

    def open(self) -> None:
        self.busy = 0

@dataclass
class Item:
    x: int
    y: int
    track_id: int

    type: int
    confidence: float

    arm_effectiveness: float
    gantry_effectiveness: float

class VisualizerEnv:
    def __init__(
        self,
        width,
        length,
        arm_positions,
        gantry_positions,
        step_count,
        items,
    ):
        self.width = width
        self.length = length
        self.arm_positions = arm_positions
        self.gantry_positions = gantry_positions
        self.step_count = step_count
        self.items = items

    def increaseStep(self):
        self.step_count += 1

    def setItems(self, newItems):
        self.items = newItems

class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__('task_planner')

        self.create_subscription(TrackedObjectArray, TRACKED_TOPIC, self.masterExecute, 1)
        self.create_subscription(String, FRANKA_STATE_TOPIC, self.handle_exec, 10)
        self.create_subscription(String, GANTRY_STATE_TOPIC, self.handle_beckhoff, 10)

        self.send_franka = self.create_publisher(TrackedObject, FRANKA_TARGET_TOPIC, 1)
        self.send_gantry = self.create_publisher(TrackedObject, GANTRY_TARGET_TOPIC, 1)

        self.n_types = 5
        #this is a hold over from the simulator for "max objects", it is irrelevant here
        #kept for compatibility purposes though
        self.max_objects = 9999

        self.step_count = 0

        self.conveyorObjectsScreenshot = {}
        self.conveyorObjectsRolling = {}
        self.convertedObjects = []
        self.previouslyTrackedIds = []

        self.franka_ready = True
        self.gantry_ready = True

        reachable_arm, reachable_gantry = self.generate_bins()

        self.arm_robot = Robot(name="arm", reachable_positions=reachable_arm, busy = 0)
        self.gantry_robot = Robot(name="gantry", reachable_positions=reachable_gantry, busy = 0)

        self.env = VisualizerEnv(NUM_BINS_X, NUM_BINS_Y, np.asarray(reachable_arm, dtype=np.int32), np.asarray(reachable_gantry, dtype=np.int32), 0, [])

        self.visualizer = ConveyorVisualizer(self.env)

    def masterExecute(self, msg):
        self.env.increaseStep()
        self.on_tracked(msg)

        observation = self.constructObservation()
        print("Current objects in motion", self.convertedObjects)
        self.env.setItems(self.convertedObjects)

        if observation == None:
            print("No observation this frame (likely due to empty conveyor), proceeding with no action")
            action = [0, 0]
        else:
            self.greedySelection(observation)
            action = [0, 0]

        self.visualizer.env = self.env
        self.visualizer.render(action)

    def on_tracked(self,msg):
        for obj in msg.objects:
            #if object has crossed middle of camera
            if obj.position.y > 0:
                #check that the object hasnt been tracked before
                if obj.track_id in self.previouslyTrackedIds:
                    print("This object has already been recorded so ignoring it")
                    return

                print("adding " + str(obj.track_id) + " to the conveyor screenshot")
                self.conveyorObjectsScreenshot[obj.track_id] = obj
                #add to the previously tracked ids to prevent dupes
                self.previouslyTrackedIds.append(obj.track_id)

    #generate bin range for each robot
    def generate_bins(self):
        arm_range = []
        gantry_range = []

        #arm bins
        for y in range(ARM_BIN_RANGE_Y[0], ARM_BIN_RANGE_Y[1] + 1):
            for x in range(1, NUM_BINS_X + 1):
                arm_range.append((x, y))

        #remove excluded bins
        for exclusion in ARM_BIN_EXCLUSION:
            arm_range.remove(exclusion)

        #gantry bins
        for y in range(GANTRY_BIN_RANGE_Y[0], GANTRY_BIN_RANGE_Y[1] + 1):
            for x in range(1, NUM_BINS_X + 1):
                gantry_range.append((x, y))
        
        #remove excluded bins
        for exclusion in GANTRY_BIN_EXLUSION:
            gantry_range.remove(exclusion)

        return arm_range, gantry_range

# Handle State Updates for Robots
    def handle_exec(self, msg):
        if msg.data == "runnin":
            self.franka_ready = False
        elif msg.data == "not runnin":
            self.franka_ready = True
        else:
            print("\n not good \n")

    def handle_beckhoff(self, msg):
        if msg.data == "runnin":
            self.gantry_ready = False
        elif msg.data == "not runnin":
            self.gantry_ready = True
        else:
            print("\n not good \n")

    def getObjectEstimatePosition(self, track_id):
        try:
            #original y should always be ~0 as it is logged in middle of camera
            originalY = self.conveyorObjectsScreenshot[track_id].position.y
            dt = (self.get_clock().now() - Time.from_msg(self.conveyorObjectsScreenshot[track_id].header.stamp)).nanoseconds * 1e-9
            dy = CONVEYOR_SPEED * dt

            newY = dy + originalY

            print("Getting new position of object ", track_id, " after ", dt, " : ", newY, " from ", originalY)

            return (self.conveyorObjectsScreenshot[track_id].position.x, newY)
        except:
            print("No object found matching requested id of " + str(track_id))

    #helper function to approximate the bin based on the x and y position
    def approximateBin(self, x, y):
        binX = math.ceil(((x * 100) + 15) / (CONVEYOR_WIDTH / NUM_BINS_X))

        if binX > NUM_BINS_X:
            binX = NUM_BINS_X

        binY = math.ceil(((y * 100)) / (CONVEYOR_LENGTH / NUM_BINS_Y))
        print("supposed biny ", y * 100)

        if binY > NUM_BINS_Y:
            binY = NUM_BINS_Y
        
        return (binX, binY)

    def greedySelection(self, observation):
        #select any cardboard or paper objects to be picked by the gantry and any others by the arm
        object_features = observation["object_features"]
        arm_reachable = observation["arm_reachable"]
        gantry_reachable = observation["gantry_reachable"]
        arm_busy = observation["arm_busy"]
        gantry_busy = observation["gantry_busy"]
        
        for i, obj in enumerate(object_features):
            print(arm_reachable[i])
            print(arm_busy)
            object_type = np.argmax(obj[2:7])
            if arm_reachable[i] and not arm_busy and (object_type == 0 or object_type == 1 or object_type == 4):
                #tell arm to grab this object
                # self.send_franka.publish(self.conveyorObjectsRolling[obj[9]])
                self.send_franka.publish(self.conveyorObjectsScreenshot[obj[9]])
                print("Trying to tell franka to grab object " + str(obj[9]))

                arm_busy = 1

                #in future add check for if the object was successfully grabbed
                #for now assume it is a success and remove the object
                print(self.conveyorObjectsScreenshot)
                self.conveyorObjectsScreenshot.pop(int(obj[9]))
                pass
            elif gantry_reachable[i] and not gantry_busy and (object_type == 3 or object_type == 2):
                #tell gantry to grab this object
                self.send_gantry.publish(self.conveyorObjectsScreenshot[obj[9]])
                print("Trying to tell gantry to grab object " + str(obj[9]))
                
                gantry_busy = 1

                #in future add check for if the object was successfully grabbed
                #for now assume it is a success and remove the object
                self.conveyorObjectsScreenshot.pop(int(obj[9]))
                pass

        pass

    def algorithmSelection(self):
        #make a decision via calling the ppo model
        pass

    #helper function to determine if a robot can reach a given point
    def is_position_reachable(self, robot, position):
        return position in robot.reachable_positions
    
    def constructObservation(self):
        self.conveyorObjectsRolling = {}
        self.convertedObjects = []

        #create a rolling object list by getting the positions of all objects that should currently be on the conveyor
        for obj in list(self.conveyorObjectsScreenshot.values()):
            newPosition = self.getObjectEstimatePosition(obj.track_id)

            if newPosition == None:
                print("Couldn't get position as object does not exist")
                return

            if (newPosition[1] * 100) > ((CONVEYOR_LENGTH)):
                #delete from list
                self.conveyorObjectsScreenshot.pop(obj.track_id, None)
                continue

            objClone = copy.deepcopy(obj)

            objClone.position.x = newPosition[0]
            objClone.position.y = newPosition[1]

            self.conveyorObjectsRolling[objClone.track_id] = objClone

        #convert the rolling objects list into the item object form for the algorithm
        for obj in list(self.conveyorObjectsRolling.values()):
            binPos = self.approximateBin(obj.position.x, obj.position.y)
            print("Bin pos of object ", obj.track_id, " at position (", obj.position.x, ",", obj.position.y, ")", " is ", "(", binPos[0], ",", binPos[1], ")")

            objClass = obj.obj_class

            #values between 0 and 1 to represent a preference for the object for each robot
            #for now will be hardcoded based on class but can be dynamically decided later on
            arm_effectiveness = 0.0
            gantry_effectiveness = 0.0

            #glass
            if objClass == 0:
                arm_effectiveness = 1.0
                gantry_effectiveness = 0.0
            #metal
            elif objClass == 1:
                arm_effectiveness = 1.0
                gantry_effectiveness = 0.0
            #paper
            elif objClass == 2:
                arm_effectiveness = 0.0
                gantry_effectiveness = 1.0
            #cardboard
            elif objClass == 3:
                arm_effectiveness = 0.2
                gantry_effectiveness = 0.8
            #plastic
            elif objClass == 4:
                arm_effectiveness = 1.0
                gantry_effectiveness = 0.0

            newItem = Item(x=binPos[0], y=binPos[1], track_id=obj.track_id, type=objClass, confidence=obj.confidence, arm_effectiveness=arm_effectiveness, gantry_effectiveness=gantry_effectiveness)

            self.convertedObjects.append(newItem)
        
        #get active state of the current robots
        #these are just classified inversely so thats why theres a not
        self.arm_robot.busy = not self.franka_ready
        self.gantry_robot.busy = not self.gantry_ready

        #define the pieces of an observation
        object_features = np.zeros(
            (
                len(self.convertedObjects),
                5 + self.n_types,
            ),
            dtype = np.float32,
        )

        object_mask = np.zeros(
            len(self.convertedObjects),
            dtype=np.int8,
        )

        arm_reachable = np.zeros(
            100,
            dtype=np.int8,
        )

        gantry_reachable = np.zeros(
            100,
            dtype=np.int8,
        )

        for i, item in enumerate(
            self.convertedObjects
        ):
            x = item.x
            y = item.y

            object_features[i, 0] = x
            object_features[i, 1] = y

            #encode classifier as one-hot data
            type_index = (
                2 + item.type
            )

            object_features[
                i,
                type_index
            ] = 1.0

            object_features[
                i,
                2 + NUM_CLASSES
            ] = item.confidence

            object_features[
                i,
                3 + NUM_CLASSES
            ] = item.arm_effectiveness

            object_features[
                i,
                4 + NUM_CLASSES
            ] = item.gantry_effectiveness

            object_features[
                i,
                4 + NUM_CLASSES
            ] = item.track_id

            object_mask[i] = 1

            if self.is_position_reachable(self.arm_robot, (item.x, item.y)):
                arm_reachable[i] = 1

            if self.is_position_reachable(self.gantry_robot, (item.x, item.y)):
                gantry_reachable[i] = 1
                
        observation = {
            "object_features": object_features,
            "object_mask": object_mask,
            "arm_reachable": arm_reachable,
            "gantry_reachable": gantry_reachable,
            "arm_busy": self.arm_robot.busy,
            "gantry_busy": self.gantry_robot.busy,
        }

        return observation


def main(args=None):
    rclpy.init(args=args)
    node = TaskPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

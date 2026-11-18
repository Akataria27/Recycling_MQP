import rclpy
import sensor_msgs_py.point_cloud2 as pc2
import numpy
import open3d
import pyrealsense2
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, Float32, Float32MultiArray, Int32
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock

from ultralytics import YOLO
import threading
import random
import time
from rclpy.time import Time

from grasp_interfaces.msg import TrackedObject, TrackedObjectArray
#for object differentiation
import cv2
MIN_PIXELS = 500          
KERNEL3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
KERNEL5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
from sensor_msgs.msg import Image

# my constants
MIN_DEPTH = .2
MAX_DEPTH = 0.80 # we think at .85 m it cant see table, try more values


# CLASSIFICATION CODE

#variables for storing found objects
detectedIds = []
trackedObjects = {}
maxMissingFrames = 15

#get model
model = YOLO("/home/merlabros2/Recycling_MQP/src/grasp_perception/grasp_perception/NewwasteDetectorLatest.pt")

#for the class names
#2 states one pulls from the model's knowledge the other is hard coded
#currnetly using hardcoded due to the model being slightly messed up this iteration
#classes = model.names
classes = ["Glass", "Metal", "Paper", "Cardboard", "Plastics"]

#Function to run the main camera loop, records found objects into the trackedObjects var



#function to retrieve a copy of trackedObjects safely outside of the camera thread


class RealSenseObstacleNode(Node):
    def __init__(self):
        super().__init__('realsense_obstacle_node')

        # IMPORTANT: Use a dedicated optical frame for the cloud
        self.declare_parameter('frame_id', 'camera_frame') 

        # RealSense Camera Setup
        self.pipeline = pyrealsense2.pipeline()
        self.config = pyrealsense2.config()
        self.config.enable_device('337122071438') # to obtain serial number: bash > rs-enumerate-devices
        self.config.enable_stream(pyrealsense2.stream.depth, 640, 480, pyrealsense2.format.z16, 30)
        self.config.enable_stream(pyrealsense2.stream.color, 640, 480, pyrealsense2.format.rgb8, 30)
        self.r = True

        self.spatial = pyrealsense2.spatial_filter()
        self.temporal = pyrealsense2.temporal_filter()

        # Start Camera
        try:
            profile = self.pipeline.start(self.config)
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            self.get_logger().info(f"RealSense Connected. Scale: {self.depth_scale}")
            depth_sensor.set_option(pyrealsense2.option.enable_auto_exposure, 0)
            depth_sensor.set_option(pyrealsense2.option.exposure, 4000.0)

            # Wait...
            for _ in range(100):
                self.pipeline.wait_for_frames()

        except Exception as e:
            self.get_logger().fatal(f"Camera Connection Failed: {e}")
            raise e

        # Processing Blocks
        self.align = pyrealsense2.align(pyrealsense2.stream.color)
        self.pointcloud = pyrealsense2.pointcloud() 

        # My Publisher
        self.tracked_pub = self.create_publisher(TrackedObjectArray, '/tracked_objects', 1)

        
        
        #Classification Start

        #tracker for the current captured screenshots 
        self.runSeed = random.randint(1, 1000)
        self.screenShotCounter = 0

        # Timer (replaces the camera thread)
        self.timer = self.create_timer(1.0/30.0, self.cameraLoop)

        self._logger.info('Successfully Initialized Obstacle Node [!]')
        

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node() # call parent Node class orignal destroy_node function
    def cameraLoop(self):
        #reset detectedIds for this frames
        detectedIds = []
        
        #wait for the next set of frames
        frames = self.pipeline.wait_for_frames()
        if frames.get_frame_timestamp_domain() == pyrealsense2.timestamp_domain.global_time:
            capture_time = Time(nanoseconds=int(frames.get_timestamp() * 1e6),   # ms -> ns
                                clock_type=self.get_clock().clock_type)
        else:
            capture_time = self.get_clock().now()   # fallback
        aligned_frames = self.align.process(frames)
        

        #extract individual frames
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        #convert to NumPy arrays
        color_image = numpy.asanyarray(color_frame.get_data())
        #convert color from rgb to bgr
        color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
        depth_image = numpy.asanyarray(depth_frame.get_data())

        print("Color:", color_image.shape, color_image.dtype)
        print("Depth:", depth_image.shape, depth_image.dtype)
                
        #run inference
        results = model.track(
            source=color_image,
            persist=True,
            #device = "cpu"
        )
        
        #Process detections
        for result in results:
            for box in result.boxes:
                if box.id is None:
                    continue
                
                trackId = int(box.id)
                cls = int(box.cls)
                confidence = float(box.conf.item())
                
                detectedIds.append(trackId)

                x1, y1, x2, y2 = map(int, box.xyxy[0])

                #get the center of detected object
                centerX = (x1 + x2) // 2
                centerY = (y1 + y2) // 2
                
                #get distance at center
                distance = depth_frame.get_distance(centerX, centerY)


                h_img, w_img = depth_image.shape
                rx1, ry1 = max(x1, 0), max(y1, 0)
                rx2, ry2 = min(x2, w_img), min(y2, h_img)
                roi = depth_image[ry1:ry2, rx1:rx2] * self.depth_scale
                mask = ((roi >= MIN_DEPTH) & (roi <= MAX_DEPTH)).astype(numpy.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL3)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL5)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # keep the last angle if the fit fails
                angle = trackedObjects.get(trackId, {}).get("angle", 0.0)
                if contours:
                    c = max(contours, key=cv2.contourArea)
                    if cv2.contourArea(c) > 100:   # tune
                        (_, _), (rw, rh), rect_ang = cv2.minAreaRect(c)
                        long_ang = rect_ang if rw >= rh else rect_ang + 90.0
                        grip = numpy.deg2rad(long_ang + 90.0)            # gripper closes across the short axis
                        angle = float((grip + numpy.pi/2) % numpy.pi - numpy.pi/2)   # wrap to [-pi/2, pi/2)
                #get the distance of the object from the center of the camera frame
                intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
                cameraCoordinate = pyrealsense2.rs2_deproject_pixel_to_point(intrinsics, [centerX, centerY], distance)
                            
                #convert to global coordinate (relative to arm)
                globalCoordinate = [cameraCoordinate[0], cameraCoordinate[1], cameraCoordinate[2]]

                #roboflow messed with my labels so this is a temp fix for this model version
                #comment out in future
                correctedClass = -1

                match cls:
                    case 0:
                        correctedClass = 3
                    case 1:
                        correctedClass = 0
                    case 2:
                        correctedClass = 1
                    case 3:
                        correctedClass = 2
                    case 4:
                        correctedClass = 4
                
                #add to tracking if new, or update location if not new
                trackedObjects[trackId] = {
                    "box": (x1, y1, x2, y2),
                    "class": correctedClass,
                    "confidence": confidence,
                    "center": (centerX, centerY),
                    "coordinate": (globalCoordinate[0], globalCoordinate[1], globalCoordinate[2]),
                    "stamp": capture_time,
                    "angle": angle,
                    "missingFrames": 0  
                }

        for track_id in list(trackedObjects.keys()):

            if track_id not in detectedIds:
                    trackedObjects[track_id]["missingFrames"] += 1

                    #remove object if it has been missing too long
                    if trackedObjects[track_id]["missingFrames"] > maxMissingFrames:
                        del trackedObjects[track_id]
        
        #annotated image
        annotated = color_image.copy()
        
        #draw boxes
        objectsToDraw = trackedObjects.copy()
            
        for track_id, obj in objectsToDraw.items():

            x1, y1, x2, y2 = obj["box"]
            cls = obj["class"]
            confidence = obj["confidence"]
            missing = obj["missingFrames"]
            globalCoordinate = obj["coordinate"]

            #get class name
            class_name = classes[cls]

            #different color if YOLO did not detect it this frame
            if missing == 0:
                #green if found
                color = (0, 255, 0)
            else:
                #yellow if remembered
                color = (0, 255, 255)

            #draw bounding box
            cv2.rectangle(
                annotated,
                (x1, y1),
                (x2, y2),
                color,
                2
            )

            #label
            label = f"{class_name} ID:{track_id}"

            if missing > 0:
                label += f" ({missing})"

            cv2.putText(
                annotated,
                label,
                (x1, max(y1 - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2
            )
            
            #draw center
            centerX = (x1 + x2) // 2
            centerY = (y1 + y2) // 2
            
            coordinateLabel = "Coord: (" + str(round(globalCoordinate[0], 3)) + ", " + str(round(globalCoordinate[1], 3)) + ", " + str(round(globalCoordinate[2], 3)) +")"
            
            #label the coordinate of the object
            cv2.putText(
                annotated,
                coordinateLabel,
                (x1, max(y2 - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2
            )
            
            cv2.circle(
                annotated,
                (centerX, centerY),
                4,
                color,
                -1
            )

        #display color
        cv2.imshow("Color", color_image)

        #display depth
        depth_display = cv2.convertScaleAbs(
            depth_image,
            alpha=0.03
        )
        cv2.imshow("Depth", depth_display)
        
        #display annotated
        cv2.imshow("annotated: ", annotated)

        #press q to quit
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            rclpy.shutdown()
            return
        
        if key & 0xFF == ord('c'):
            print("Screen shot taken")
            success = cv2.imwrite("/home/merlabros2/Recycling_MQP/src/grasp_perception/customDataCollection/capture" + str(self.runSeed) + str(self.screenShotCounter) + ".jpg", color_image)
            self.screenShotCounter += 1

        arr = TrackedObjectArray()
        arr.header.stamp = capture_time.to_msg()
        arr.header.frame_id = self.get_parameter('frame_id').value
        for tid, o in trackedObjects.items():
            m = TrackedObject()
            m.header.stamp = o["stamp"].to_msg()
            m.header.frame_id = arr.header.frame_id
            m.track_id = tid
            m.obj_class = o["class"]
            m.confidence = o["confidence"]
            m.position.x, m.position.y, m.position.z = o["coordinate"]
            m.angle = o["angle"]
            m.missing_frames = o["missingFrames"]
            arr.objects.append(m)
        self.tracked_pub.publish(arr)
    def getTrackedObjects(self):
        return trackedObjects.copy()

def main(args=None):

    rclpy.init(args=args)
    node = RealSenseObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
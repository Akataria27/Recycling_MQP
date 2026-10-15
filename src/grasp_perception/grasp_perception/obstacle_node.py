import rclpy
import sensor_msgs_py.point_cloud2 as pc2
import numpy
import open3d
import pyrealsense2
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, Float32, Float32MultiArray, Int32
from sensor_msgs.msg import Image

from ultralytics import YOLO
import threading
import random
import time

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

#threading
trackedObjectsLock = threading.Lock()
stopEvent = threading.Event()


#get model
model = YOLO("/home/merlabros2/Recycling_MQP/src/grasp_perception/grasp_perception/wasteDetectorLatest.pt")


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

        self.frames = None
        self.trackingThread = threading.Thread(
                target = self.cameraLoop,
                daemon = True
            )
        self.trackingThread.start()

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

        # --- ROS2 ---
        # My Subscription
        self.subtojesus = self.create_subscription(Float32MultiArray, '/coord', self.command_cb, 1)

        # My Publisher
        self.pub = self.create_publisher(PointCloud2, '/object_cloud', 1)
        self.pub1 = self.create_publisher(Float32, '/angle', 1)

        self.gantry_publish = self.create_publisher(Float32MultiArray, '/gantry_object_cloud', 1)
        
        # My Timers
        self.timer = self.create_timer(1.0/30.0, self.timer_cb)
        
        #Classification Start
        
        self.type = self.create_publisher(Int32, '/type', 1)
        self.coord = self.create_publisher(Float32MultiArray, '/coord', 1)
        self.object = None
        self.obj_type = None
        # Coordinates

        self.x_obj = None
        self.y_obj = None

    def command_cb(self, msg):
        print(float(f"{msg.data[0]:.5f}"), float(f"{msg.data[1]:.5f}"))
        # self.x_obj = float(f"{msg.data[0]:.5f}")
        # self.y_obj = float(f"{msg.data[1]:.5f}")

    def timer_cb(self):
        # 1. Get Frames
        self.frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(self.frames)
        depth_frame = aligned_frames.get_depth_frame()

        if not depth_frame: 
            self.get_logger().fatal('not good')
            return

        depth_frame = self.temporal.process(depth_frame)
        depth_frame = self.spatial.process(depth_frame)
        depth_frame = depth_frame.as_depth_frame() 

        # 2. Generate Pyrealsense2 Point Cloud
        points = self.pointcloud.calculate(depth_frame)

        if points.size() == 0:
            print('0')
            return
        
        # 3. Convert to NumPy
        w, h = depth_frame.get_width(), depth_frame.get_height()
        vtx = numpy.asanyarray(points.get_vertices())
        verts = vtx.view(numpy.float32).reshape(h, w, 3)
        x = verts[:,:,0]
        y = verts[:, :, 1]
        z = verts[:, :, 2] # array of every pixel, removes those not in Z clamp
        mask = (
            (x >= -0.23) & (x <= 0.37) & 
            (z >= MIN_DEPTH) & (z <= MAX_DEPTH)
        ).astype(numpy.uint8)
        
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL3)    # Prevent noise, aka small gaps in objects do not get treated as their own objects 
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL5)
        
        # 5. Label connected blobs
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8) # 8 means mode where adjacent pixels and diagonal pixels are touching. 
        objects = []
        for i in range(1, n): #start at 1 because conveyor/background is biggest object
            if stats[i, cv2.CC_STAT_AREA] < MIN_PIXELS: # ignore things too small
                continue
            pts = verts[labels == i]     #sorts through all the points and finds all poins related to object num i, these lables are made in #5         
            pts = pts[pts[:, 2] > 0]              # drop invalid vertices
            if len(pts) < MIN_PIXELS // 2:
                continue
            objects.append({'points': pts, 'centroid': pts.mean(axis=0)})
        
        if not objects:
            return



        object_list = self.getTrackedObjects()
        if not object_list:
            return
        objectCoords = []
        objectClasses = []

        for trackid, obj in object_list.items():
            objectCoords.append(obj["coordinate"])
            objectClasses.append(obj["class"])

        if objectClasses:
            if len(objectCoords) > 0:
                msg = Float32MultiArray(data=objectCoords[0])
                self.object = objectCoords[0]
                
            if len(objectClasses) > 0:
                self.obj_type = objectClasses[0]
                
        
        # 6. Pick one: nearest to the commanded coord, else the biggest blob

        if self.object is not None:
            
            oz, oy, ox = self.object
            if oy < -0.1:
                return
            target = min(objects, key=lambda o:
                        (o['centroid'][0] - ox) ** 2 +
                        (o['centroid'][1] - oy) ** 2)
        else:
            target = max(objects, key=lambda o: len(o['points']))

        obj_pts = target['points']
        self.type.publish(Int32(data=int(self.obj_type)))



        # 7. PCA on that object alone
        xy = obj_pts[:, :2].astype(float)
        cov = numpy.cov(xy - xy.mean(axis=0), rowvar=False)
        evals, evecs = numpy.linalg.eigh(cov)
        minor = evecs[:, 0]
        yaw_grip = numpy.arctan2(minor[1], minor[0])
        self.pub1.publish(Float32(data=float(yaw_grip)))

        # 8. Publish
        pcd = open3d.geometry.PointCloud()
        pcd.points = open3d.utility.Vector3dVector(obj_pts.astype(numpy.float64))
        self.publish_cloud(pcd)

        if len(objectCoords) <= 1:
            return
        

        #self.get_logger().info(f"{len(objects)} objects, target has {len(obj_pts)} points")

    def publish_cloud(self, pcd):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.get_parameter('frame_id').value 
        
        points = numpy.asarray(pcd.points)

        if points.shape[0] == 0:
            print('\n There are no points in the cloud!!!!!!')
            return

        # Create and Publish Open3D Point Cloud Data As sensor_msgs_py.point_cloud2
        msg = pc2.create_cloud_xyz32(header, points)
        self.pub.publish(msg)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node() # call parent Node class orignal destroy_node function
    def cameraLoop(self):
        #tracker for the current captured screenshots 
        runSeed = random.randint(1, 1000)
        screenShotCounter = 0
        
        while not stopEvent.is_set():
            if self.frames == None:
                print("frames hasnt been updated")
                continue
            #reset detectedIds for this frames
            detectedIds = []
            
            #wait for the next set of frames
            

            #extract individual frames
            color_frame = self.frames.get_color_frame()
            depth_frame = self.frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

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
                                
                    #get the distance of the object from the center of the camera frame
                    intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
                    cameraCoordinate = pyrealsense2.rs2_deproject_pixel_to_point(intrinsics, [centerX, centerY], distance)
                                
                    #convert to global coordinate (relative to arm)
                    globalCoordinate = [cameraCoordinate[2], cameraCoordinate[1], cameraCoordinate[0]]
                    
                    with trackedObjectsLock:
                        #add to tracking if new, or update location if not new
                        trackedObjects[trackId] = {
                            "box": (x1, y1, x2, y2),
                            "class": cls,
                            "confidence": confidence,
                            "center": (centerX, centerY),
                            "coordinate": (globalCoordinate[0], globalCoordinate[1], globalCoordinate[2]),
                            "missingFrames": 0
                        }

            with trackedObjectsLock:
                for track_id in list(trackedObjects.keys()):

                    if track_id not in detectedIds:
                            trackedObjects[track_id]["missingFrames"] += 1

                            #remove object if it has been missing too long
                            if trackedObjects[track_id]["missingFrames"] > maxMissingFrames:
                                del trackedObjects[track_id]
            
            #annotated image
            annotated = color_image.copy()
            
            #draw boxes
            #get a copy first so that the lock isnt held for too long
            with trackedObjectsLock:
                objectsToDraw = trackedObjects.copy()
                
            for track_id, obj in objectsToDraw.items():

                x1, y1, x2, y2 = obj["box"]
                cls = obj["class"]
                confidence = obj["confidence"]
                missing = obj["missingFrames"]
                globalCoordinate = obj["coordinate"]

                #get class name
                class_name = model.names[cls]

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
                stopEvent.set()
                break
            
            if key & 0xFF == ord('c'):
                print("Screen shot taken")
                success = cv2.imwrite("customDataCollection/capture" + str(runSeed) + str(screenShotCounter) + ".jpg", color_image)
                screenShotCounter += 1
    def getTrackedObjects(self):
        with trackedObjectsLock:
            return trackedObjects.copy()

def main(args=None):

    rclpy.init(args=args)
    node = RealSenseObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stopEvent.set()
        node.trackingThread.join()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
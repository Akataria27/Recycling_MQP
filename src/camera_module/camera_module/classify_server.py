#!/usr/bin/env python3
# classify_server.py

import sys
import os
import logging
import numpy as np
import cv2
import torch
import torchvision
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision import transforms
from PIL import Image

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

# Interface Imports
from sensor_msgs.msg import Image as RosImage
from camera_interfaces.srv import ClassifySrv
# Assuming detection_msgs is the package containing BoundingBox
from camera_interfaces import BoundingBox 

class ClassifyServer(Node):
    def __init__(self):
        super().__init__('classify_server')
        
        # Initialize Service
        self.srv = self.create_service(ClassifySrv, 'classify_waste', self.run_classifier)
        
        # Logging
        # In ROS 2, we can use the node's logger, but keeping your custom logger setup is fine too.
        self.logger_custom = logging.getLogger('infer_logger')
        self.logger_custom.setLevel(logging.DEBUG)
        
        c_handler = logging.StreamHandler(sys.stdout)
        f_handler = logging.FileHandler('infer.log')
        c_handler.setLevel(logging.DEBUG)
        f_handler.setLevel(logging.DEBUG)
        
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        c_handler.setFormatter(formatter)
        f_handler.setFormatter(formatter)
        
        self.logger_custom.addHandler(c_handler)
        self.logger_custom.addHandler(f_handler)

        self.get_logger().info("Classify Server Ready")

        # Constants
        self.CLASSES = ['__background__', 'cardboard', 'glass', 'metal', 'paper', 'plastic']
        self.NUM_CLASSES = len(self.CLASSES)
        self.confidence_threshold = 0.7

        # Model Path Handling
        # Note: In ROS 2 launch contexts, os.getcwd() often returns ~/.ros or the launch dir.
        # It is safer to use parameters or ament_index_python to find paths.
        current_dir = os.getcwd()
        print(f"Current Directory: {current_dir}")
        self.model_name = os.path.join(current_dir, 'models/mobilenet_ss_18_wd_0001_class_dataset/fasterrcnn_model.pth')

        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Load Model once at startup (Optimization)
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = self.get_model_instance_segmentation(self.NUM_CLASSES)
        
        if os.path.exists(self.model_name):
            self.model.load_state_dict(torch.load(self.model_name, map_location=self.device))
            self.model.eval().to(self.device)
            self.get_logger().info(f"Model loaded from {self.model_name}")
        else:
            self.get_logger().error(f"Model file not found at {self.model_name}")

    def get_model_instance_segmentation(self, num_classes):
        model = torchvision.models.detection.fasterrcnn_mobilenet_v3_large_fpn(pretrained=False)
        in_features = model.roi_heads.box_predictor.cls_score.in_features
        model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)
        return model

    def infer_frame(self, model, frame, device):
        transform = transforms.ToTensor()
        input_tensor = transform(frame).to(device)
        with torch.no_grad():
            outputs = model([input_tensor])[0]
        return outputs

    def run_classifier(self, request, response):
        self.get_logger().info("Incoming request...")
        
        try:
            # Convert ROS Image to CV2
            rgb_frame = self.bridge.imgmsg_to_cv2(request.rgb_image, desired_encoding="rgb8")
            pil_image = Image.fromarray(rgb_frame)

            # Run inference
            outputs = self.infer_frame(self.model, pil_image, self.device)
            
            # Extract results
            boxes = outputs['boxes'].cpu().numpy()
            labels = outputs['labels'].cpu().numpy()
            scores = outputs['scores'].cpu().numpy()

            response.boxes = []

            for box, label, score in zip(boxes, labels, scores):
                if score > self.confidence_threshold:
                    xmin, ymin, xmax, ymax = map(int, box)
                    
                    # Create the BoundingBox message and populate it
                    bbox_msg = BoundingBox()
                    bbox_msg.xmin = float(xmin)
                    bbox_msg.ymin = float(ymin)
                    bbox_msg.xmax = float(xmax)
                    bbox_msg.ymax = float(ymax)
                    bbox_msg.confidence = float(score)
                    
                    # Map the numeric label to the string class name
                    # Ensure self.CLASSES has the correct mapping order
                    class_name = self.CLASSES[int(label)]
                    bbox_msg.class_label = class_name
                    
                    response.boxes.append(bbox_msg)

            self.get_logger().info(f'Sending {len(response.boxes)} bounding boxes')
            self.logger_custom.info("Real-time detection ended.")

        except Exception as e:
            self.get_logger().error(f"Error during classification: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ClassifyServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import sys
import os
import logging
import torch
import torchvision
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision import transforms
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pyrealsense2 as rs


# from train import WasteDataset
# from torch.utils.data import DataLoader, Dataset
from torchvision import transforms
# import random

import cv2
from PIL import Image
import numpy as np

import rospy
from std_msgs.msg import Float64MultiArray, Float64

class InferLive:
    def __init__(self):
        rospy.init_node("infer_live")
        rospy.sleep(1.0)
        rospy.loginfo("Infer Live Node Ready")

        # rospy.Subscriber('/classify', Float64, self.main())
        # self.box_publisher = rospy.Publisher('/objects_detected', Float64MultiArray, queue_size=10)


        # Set up logging
        self.logger = logging.getLogger('infer_logger')
        self.logger.setLevel(logging.DEBUG)

        # Create handlers for console and file logging
        c_handler = logging.StreamHandler(sys.stdout)
        f_handler = logging.FileHandler('infer.log')

        c_handler.setLevel(logging.DEBUG)
        f_handler.setLevel(logging.DEBUG)

        # Create formatters and add them to handlers
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        c_handler.setFormatter(formatter)
        f_handler.setFormatter(formatter)

        # Add handlers to the logger
        self.logger.addHandler(c_handler)
        self.logger.addHandler(f_handler)

        # Constants
        # CLASSES = ['__background__', 'recycling', 'nonrecycling']  # Include background
        # CLASSES = ['__background__', 'nonplastic', 'plastic']
        self.CLASSES = ['__background__', 'cardboard', 'glass', 'metal', 'paper', 'plastic']

        self.NUM_CLASSES = len(self.CLASSES)


        current_dir = os.getcwd()
        print(f"current dir: {current_dir}")
        # current_dir = os.path.join(current_dir, "src/robots_for_recycling")
        self.model_name = os.path.join(current_dir, 'models/mobilenet_ss_18_wd_0001_class_dataset/fasterrcnn_model.pth')
        # self.model_name = 'models/mobilenet_ss_18_wd_0001/fasterrcnn_mobilenet_ss_18_wd_0001.pth'
        self.confidence_threshold = 0.7

    def get_model_instance_segmentation(self, num_classes):
        model = torchvision.models.detection.fasterrcnn_mobilenet_v3_large_fpn(pretrained=False)
        self.logger.info("Initialized Faster R-CNN model with MobileNetV3 backbone.")

        # Get the number of input features for the classifier
        in_features = model.roi_heads.box_predictor.cls_score.in_features

        # Replace the pre-trained head with a new one
        model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)
        self.logger.info("Modified the model's box predictor to accommodate the new number of classes.")

        return model

    def infer_frame(self, model, frame, device):
        transform = transforms.ToTensor()
        input_tensor = transform(frame).to(device)

        with torch.no_grad():
            outputs = model([input_tensor])[0]
        return outputs

    def draw_bounding_boxes(self, frame, boxes, labels, scores, threshold=0.5): 
        for box, label, score in zip(boxes, labels, scores):
            if score > threshold:
                xmin, ymin, xmax, ymax = map(int, box)
                class_name = self.CLASSES[label]
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                cv2.putText(frame, f'{class_name}: {score:.2f}', (xmin, ymin - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return frame
    
    def capture_frames(self):
        
        # Create a pipeline
        pipeline = rs.pipeline()

        # Create a config and enable the bag file
        config = rs.config()

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

        pipeline.start(config)

        # Create an align object to align color frames to depth frames
        align_to = rs.stream.color
        align = rs.align(align_to)
        
        try:
            while True:

                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                color_frame = aligned_frames.first(rs.stream.color)

                # Keep looping until valid depth and color frames are received.
                if not color_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())
                break
        
        except RuntimeError as e:
            print(f"Error occurred: {e}")
        finally:
            pipeline.stop()
            return color_image

    def main(self):
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        model = self.get_model_instance_segmentation(self.NUM_CLASSES)
        # model.load_state_dict(torch.load('fasterrcnn_model.pth', map_location=device))
        model.load_state_dict(torch.load(self.model_name, map_location=device))

        model.eval().to(device)

        # # Open a connection to the webcam
        # cv2.VideoCapture()
        # for i in range(5):  # Testing indices from 0 to 4
        #     cap = cv2.VideoCapture(i)
        #     if cap.isOpened():
        #         print(f"Camera found at index {i}")
        #         # break
        #     cap.release()
        # else:
        #     print("No camera found")

        # rospy.sleep(5)

        # cap = cv2.VideoCapture(0)
        # # cap = cv2.VideoCapture(0)
        
        # if not cap.isOpened():
        #     self.logger.error("Failed to open webcam.")
        #     sys.exit(1)

        self.logger.info("Starting real-time object detection. Press 'q' to quit.")

        while True:
            try:
                rgb_frame = self.capture_frames()

                # Convert frame to RGB and PIL Image
                # rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                pil_image = Image.fromarray(rgb_frame)

                # Run inference
                outputs = self.infer_frame(model, pil_image, device)
                
                # Get the boxes, labels, and scores
                boxes = outputs['boxes'].cpu().numpy()
                labels = outputs['labels'].cpu().numpy()
                scores = outputs['scores'].cpu().numpy()
                # ret, frame = cap.read()
                # if not ret:
                #     self.logger.error("Failed to grab frame.")
                #     sys.exit(1)
                #     break

                # # Convert frame to RGB and PIL Image
                # rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                # pil_image = Image.fromarray(rgb_frame)

                # # Run inference
                # outputs = self.infer_frame(model, pil_image, device)
                
                # # Get the boxes, labels, and scores
                # boxes = outputs['boxes'].cpu().numpy()
                # labels = outputs['labels'].cpu().numpy()
                # scores = outputs['scores'].cpu().numpy()

                # Draw the bounding boxes and labels on the frame
                frame = self.draw_bounding_boxes(rgb_frame, boxes, labels, scores, threshold=self.confidence_threshold)

                yolo_v5_array = []
                i = 0
                for box, label, score in zip(boxes, labels, scores):
                    xmin, ymin, xmax, ymax = map(int, box)
                    width = xmax - xmin
                    height = ymax - ymin
                    center_x = (xmax + xmin) / 2
                    center_y = (ymax + ymin) / 2
                    yolo_v5_array.append([label, center_x, center_y, width, height])
                    i += 1
                # yolo_v5_msg = Float64MultiArray()
                # yolo_v5_msg.data = yolo_v5_array
                # self.box_publisher.publish(yolo_v5_msg)

                # Display the frame
                cv2.imshow('Real-Time Waste Detector', frame)

                # Press 'q' to exit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except:
                print("missed a frame")

        # cap.release()
        cv2.destroyAllWindows()
        self.logger.info("Real-time detection ended.")

    # def run(self):
    #     rospy.spin()


if __name__ == '__main__':
    
    # InferLive().run()
    inferLive = InferLive()
    inferLive.main()
#!/usr/bin/env python3
# train.py

import os
import sys
import logging
import torch
import torchvision
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torch.utils.data import DataLoader, Dataset
from torchvision import transforms
from PIL import Image
import numpy as np

from torchvision.ops import box_iou
from torch.utils.tensorboard import SummaryWriter
from pathlib import Path


# Set up logging
logger = logging.getLogger('train_logger')
logger.setLevel(logging.DEBUG)

# Create handlers for console and file logging
c_handler = logging.StreamHandler(sys.stdout)
f_handler = logging.FileHandler('train.log')

c_handler.setLevel(logging.DEBUG)
f_handler.setLevel(logging.DEBUG)

# Create formatters and add them to handlers
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
c_handler.setFormatter(formatter)
f_handler.setFormatter(formatter)

# Add handlers to the logger
logger.addHandler(c_handler)
logger.addHandler(f_handler)

# Constants
# CLASSES = ['__background__', 'recycling', 'nonrecycling']  # Include background
# CLASSES = ['__background__', 'nonplastic', 'plastic']
CLASSES = ['__background__', 'cardboard', 'glass', 'metal', 'paper', 'plastic']

NUM_CLASSES = len(CLASSES)

# Dataset class
class WasteDataset(Dataset):
    def __init__(self, root, transforms=None):
        self.root = root
        self.transforms = transforms
        self.images_dir = os.path.join(root, 'images')
        self.labels_dir = os.path.join(root, 'labels')
        self.image_files = sorted([f for f in os.listdir(self.images_dir) if f.endswith('.jpg')])
        logger.info(f"Initialized dataset with {len(self.image_files)} images.")

    def __len__(self):
        return len(self.image_files)

    def __getitem__(self, idx):
        # Load images and masks
        img_name = self.image_files[idx]
        img_path = os.path.join(self.images_dir, img_name)
        label_path = os.path.join(self.labels_dir, os.path.splitext(img_name)[0] + '.txt')

        # Load image
        try:
            img = Image.open(img_path).convert("RGB")
        except Exception as e:
            logger.error(f"Error loading image {img_path}: {e}")
            raise

        # Get bounding boxes and labels
        boxes = []
        labels = []
        if os.path.exists(label_path):
            with open(label_path, 'r') as f:
                for line in f:
                    try:
                        class_id, cx, cy, w, h = map(float, line.strip().split())
                        class_id = int(class_id) + 1  # Adjust for background class at index 0
                        labels.append(class_id)

                        # Convert from normalized center coordinates to absolute coordinates
                        img_width, img_height = img.size
                        x_center = cx * img_width
                        y_center = cy * img_height
                        box_width = w * img_width
                        box_height = h * img_height

                        xmin = x_center - box_width / 2
                        ymin = y_center - box_height / 2
                        xmax = x_center + box_width / 2
                        ymax = y_center + box_height / 2

                        boxes.append([xmin, ymin, xmax, ymax])
                    except Exception as e:
                        logger.error(f"Error processing label in {label_path}: {e}")
        else:
            logger.warning(f"Label file not found for image {img_name}")

        boxes = torch.as_tensor(boxes, dtype=torch.float32)
        labels = torch.as_tensor(labels, dtype=torch.int64)

        target = {}
        target["boxes"] = boxes
        target["labels"] = labels
        target["image_id"] = torch.tensor([idx])

        if self.transforms:
            img = self.transforms(img)

        return img, target
    

# Training function
def train():
    # Hyperparameters
    model_params = Params()
    num_epochs = model_params.num_epochs
    batch_size = model_params.batch_size
    learning_rate = model_params.lr
    momentum = model_params.momentum
    weight_decay = model_params.weight_decay

    logger.info(f"Starting training: batch_size={batch_size}, learning_rate={learning_rate}, num_epochs={num_epochs}")

    # Data transformations
    transform = transforms.Compose([
        transforms.ToTensor(),
    ])

    # Dataset and DataLoader 
    ###################### Change the dataset path here ######################
    dataset = WasteDataset(
        # root='c:/Users/chase/OneDrive/Documents/Grad/Robots_for_Recycling/waste_detector/waste_detector_repo/dataset/train',
        # root='c:/Users/chase/OneDrive/Documents/Grad/Robots_for_Recycling/waste_detector/waste_detector_repo/our_dataset/train',

        # root='c:/Users/chase/OneDrive/Documents/Grad/Robots_for_Recycling/waste_detector/waste_detector_repo/plastic_and_metal_dataset/train',
        root='c:/Users/chase/OneDrive/Documents/Grad/Robots_for_Recycling/waste_detector/waste_detector_repo/class_dataset/train',


        transforms=transform
    )

    val_dataset = WasteDataset(
        # root='c:/Users/chase/OneDrive/Documents/Grad/Robots_for_Recycling/waste_detector/waste_detector_repo/dataset/valid',
        # root='c:/Users/chase/OneDrive/Documents/Grad/Robots_for_Recycling/waste_detector/waste_detector_repo/our_dataset/valid',

        # root='c:/Users/chase/OneDrive/Documents/Grad/Robots_for_Recycling/waste_detector/waste_detector_repo/plastic_and_metal_dataset/valid',
        root='c:/Users/chase/OneDrive/Documents/Grad/Robots_for_Recycling/waste_detector/waste_detector_repo/class_dataset/valid',


        transforms=transform
    )

    # Split dataset into train and test sets
    indices = torch.randperm(len(dataset)).tolist()
    dataset_train = torch.utils.data.Subset(dataset, indices)

    data_loader = DataLoader(
        dataset_train, batch_size=batch_size, shuffle=True, num_workers=4,
        collate_fn=custom_collate_fn # lambda x: tuple(zip(*x))
    )

    val_data_loader = DataLoader(
        val_dataset, batch_size=len(val_dataset), shuffle = False, num_workers=4,
        collate_fn=custom_collate_fn
    )

    logger.info("DataLoader created.")

    # Get the model using our helper function
    model = get_model_instance_segmentation(NUM_CLASSES, model_params, logger)
 
    # Check if GPU is available
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model.to(device)
    logger.info(f"Model loaded and moved to device: {device}")

    # Construct an optimizer
    params = [p for p in model.parameters() if p.requires_grad]
    optimizer = torch.optim.SGD(params, lr=learning_rate,
                                momentum=momentum, weight_decay=weight_decay)

    # Learning rate scheduler
    lr_scheduler = torch.optim.lr_scheduler.StepLR(optimizer,
                                                   step_size=model_params.lr_step_size,
                                                   gamma=model_params.lr_gamma)

    start_epoch = 0
    resume_training = model_params.resume_training
    checkpoint_path = os.path.join("checkpoints", model_params.run_title, f"checkpoint.pth")

    if resume_training and os.path.exists(checkpoint_path):
        print("reloading model from last checkpoint")
        checkpoint = torch.load(checkpoint_path)
        model.load_state_dict(checkpoint["model"])
        start_epoch = checkpoint["epoch"] + 1
        optimizer.load_state_dict(checkpoint["optimizer"])
        lr_scheduler.load_state_dict(checkpoint["lr_scheduler"])
        assert model_params == checkpoint["model_params"]

    # Path(os.path.join("checkpoints_val", params.name)).mkdir(parents=True, exist_ok=True)
    Path(os.path.join("checkpoints", model_params.run_title)).mkdir(parents=True, exist_ok=True)

    print(f"start epoch: {start_epoch}")
    print("Training")
    for epoch in range(start_epoch, num_epochs):
        model.train()
        epoch_loss = 0
        for images, targets in data_loader: 
            images = list(image.to(device) for image in images)
            targets = [{k: v.to(device) for k, v in t.items()} for t in targets]

            try:
                loss_dict = model(images, targets)
                losses = sum(loss for loss in loss_dict.values())
            except Exception as e:
                logger.error(f"Error during model training: {e}")
                continue

            optimizer.zero_grad()
            losses.backward()
            optimizer.step()

            batch_loss = losses.item()
            epoch_loss += batch_loss
            # logger.debug(f"Epoch [{epoch+1}/{num_epochs}], Batch Loss: {batch_loss:.4f}")

        # Save checkpoint
        checkpoint = {
            "model": model.state_dict(),
            "optimizer": optimizer.state_dict(),
            "lr_scheduler": lr_scheduler.state_dict(),
            "epoch": epoch,
            "model_params": model_params
        }
        torch.save(checkpoint, os.path.join("checkpoints", model_params.run_title, f"model_{epoch}.pth"))
        torch.save(checkpoint, os.path.join("checkpoints", model_params.run_title, f"checkpoint.pth"))

        # Update the learning rate
        lr_scheduler.step()
        avg_loss = epoch_loss / len(data_loader)
        logger.info(f"Epoch [{epoch+1}/{num_epochs}] Average Loss: {avg_loss:.4f}")
        Path(os.path.join("runs/", f"{model_params.run_title}/", 'train')).mkdir(parents=True, exist_ok=True)
        writer_train = SummaryWriter(f'runs/{model_params.run_title}/train')
        writer_train.add_scalar('average_training_loss', avg_loss, epoch)

        # Get accuracy for this epoch
        model.eval()
        Path(os.path.join("runs/", f"{model_params.run_title}/", 'validation')).mkdir(parents=True, exist_ok=True)
        writer = SummaryWriter(f'runs/{model_params.run_title}/validation')
        compute_accuracy(model, val_data_loader, device, epoch, writer)
        
        

    # Save the trained model
    Path(os.path.join("models", model_params.run_title)).mkdir(parents=True, exist_ok=True)
    torch.save(model.state_dict(), os.path.join('models', model_params.run_title, 'fasterrcnn_model.pth'))

    # Define the file path where parameters will be saved
    file_path = os.path.join("models", f"{model_params.run_title}", "parameters.txt")
    parameters = model_params.get_params()

    # Write the parameters to the text file
    with open(file_path, 'x') as f:
        for param, value in parameters.items():
            # Write the parameter name and value in the format "param_name = value"
            f.write(f"{param} = {value}\n")
    logger.info("Training completed, params saved, and model saved to 'fasterrcnn_model_" + model_params.run_title + ".pth'.")

# def get_final_trained_model_accuracy(model, data_loader):

def compute_iou(box1, box2):
    # Compute IoU between two sets of boxes
    iou = box_iou(box1, box2)
    return iou

def compute_accuracy(model, data_loader, device, epoch, writer, iou_threshold=0.5):
    model.to(device)
    correct_detections = 0
    total_detections = 0

    with torch.no_grad():
        for images, targets in data_loader:
            images = list(image.to(device) for image in images)
            targets = [{k: v.to(device) for k, v in t.items()} for t in targets]
            outputs = model(images)  # Get predictions

            for output, target in zip(outputs, targets):
                pred_boxes = output['boxes']
                gt_boxes = target['boxes']
                pred_labels = output['labels']
                gt_labels = target['labels']

                # Calculate IoU for each prediction-gt pair
                ious = compute_iou(pred_boxes, gt_boxes)

                for i, iou_row in enumerate(ious): #ENSURE LOOPS OVER LENGTH OF IOUS, NOT EACH ELEMENT IN EACH ROW OF IOUS
                    max_iou, max_iou_idx = iou_row.max(0)  # Max IoU for the i-th predicted box

                    # Should handle if extra boxes are made
                    if max_iou > iou_threshold and pred_labels[i] == gt_labels[max_iou_idx]:
                        correct_detections += 1  # Correct label and IoU match
                    total_detections += 1

    # Calculate accuracy based on accuracy of drawn box and classification made
    accuracy = correct_detections / total_detections if total_detections > 0 else 0
    writer.add_scalar('validation_accuracy',
                                    100*accuracy,
                                    epoch)

    return accuracy

def custom_collate_fn(batch):
    return tuple(zip(*batch))

def get_model_instance_segmentation(num_classes, model_params, logger):
    # Load a pre-trained model for classification and return
    # a model ready for fine-tuning

    if model_params.name == "mobilenet_backbone":
        model = torchvision.models.detection.fasterrcnn_mobilenet_v3_large_fpn(pretrained=True)
    elif model_params.name == "resnet_backbone":
        model = torchvision.models.detection.fasterrcnn_resnet50_fpn_v2(pretrained=True)
    else: logger.error(f"model_params.name is: {model_params.name}, which does not match any implemented models")

    logger.info(f"Loaded pre-trained Faster R-CNN model with {model_params.name}.")

    # Get the number of input features for the classifier
    in_features = model.roi_heads.box_predictor.cls_score.in_features

    # Replace the pre-trained head with a new one
    model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)
    logger.info("Modified the model's box predictor to accommodate the new number of classes.")

    return model

class Params:
    def __init__(self):
        self.num_epochs = 30
        self.batch_size = 4
        self.lr = 0.005
        self.momentum = 0.9
        self.weight_decay = 0.0001 # 0.0005
        self.lr_step_size = 18 #3
        self.lr_gamma = 0.1
        self.name = "resnet_backbone" # mobilenet_backbone #resnet_backbone
        self.resume_training = True
        self.run_title = "resnet_ss_18_wd_0001_class_dataset"

    def get_params(self):
        parameters = {
            'num_epochs': self.num_epochs,
            'batch_size': self.batch_size,
            'lr': self.lr,
            'momentum': self.momentum,
            'weight_decay': self.weight_decay,
            'lr_step_size': self.lr_step_size,
            'lr_gamma': self.lr_gamma,
            'name': self.name,
            'resume_training': self.resume_training,
            'run_title': self.run_title
        }
        return parameters

    def __repr__(self):
        return str(self.__dict__)
    
    def __eq__(self, other):
        return self.__dict__ == other.__dict__

if __name__ == '__main__':
    """
    Usage: 
        Running this code will create and save model to a "model" folder in the current directory will create and save 
        model checkpoints to a "checkpoint/model.run_title" directory
            * If training is killed before finishing, you can resume model training by rerunning this script with the same 
              hyperparameters
        
        Change the model hyperparameters and name in the "Params" method above before running

        Will run on GPU if CUDA is available, else CPU
    
    """
    
    train()

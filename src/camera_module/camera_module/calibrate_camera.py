import logging
import os
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy import optimize
# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

from camera import RealSenseCamera


class Calibration:
    def __init__(self,
                 cam_id,
                 calib_grid_step,
                 checkerboard_offset_from_tool,
                 workspace_limits):
        """
        Initialize the Calibration object.

        :param cam_id: Serial number of the camera to connect to.
        :param calib_grid_step: Step size for the calibration grid.
        :param checkerboard_offset_from_tool: Offset of the checkerboard from the tool.
        :param workspace_limits: Workspace limits in robot coordinates.
        """
        self.calib_grid_step = calib_grid_step
        self.checkerboard_offset_from_tool = checkerboard_offset_from_tool

        # Workspace limits are defined as [x_min, x_max], [y_min, y_max], [z_min, z_max]
        self.workspace_limits = workspace_limits

        # Initialize the RealSense camera
        self.camera = RealSenseCamera(device_id=cam_id)
        
        # Lists to store measured points, observed points, and observed pixel coordinates
        self.measured_pts = []
        self.observed_pts = []
        self.observed_pix = []

        # Transformation matrix from camera to world coordinates
        self.camera2world = np.eye(4)

        # Paths for communication with the robot
        homedir = os.path.join(os.path.expanduser('~'), "grasp-comms")
        self.move_completed = os.path.join(homedir, "move_completed.npy")
        self.tool_position = os.path.join(homedir, "tool_position.npy")

    @staticmethod
    def _get_rigid_transform(A, B):
        """
        Estimate the rigid transform (rotation and translation) between two sets of points.

        :param A: Source points.
        :param B: Target points.
        :return: Rotation matrix (R) and translation vector (t).
        """
        assert len(A) == len(B)

        # Compute centroids of both point sets
        N = A.shape[0]
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)

        # Center the points around the origin
        AA = A - np.tile(centroid_A, (N, 1))
        BB = B - np.tile(centroid_B, (N, 1))

        # Compute the covariance matrix
        H = np.dot(np.transpose(AA), BB)

        # Perform Singular Value Decomposition (SVD)
        U, S, Vt = np.linalg.svd(H)

        # Compute the rotation matrix
        R = np.dot(Vt.T, U.T)

        # Handle the special case of reflection
        if np.linalg.det(R) < 0:
            Vt[2, :] *= -1
            R = np.dot(Vt.T, U.T)

        # Compute the translation vector
        t = np.dot(-R, centroid_A.T) + centroid_B.T
        return R, t

    def _get_rigid_transform_error(self, z_scale):
        """
        Calculate the Root Mean Square Error (RMSE) of the rigid transform.

        :param z_scale: Scaling factor for the depth values.
        :return: RMSE of the rigid transform.
        """
        # Apply the z-scale to the observed points
        observed_z = np.squeeze(self.observed_pts[:, 2:] * z_scale)
        observed_x = np.multiply(np.squeeze(self.observed_pix[:, [0]]) - self.camera.intrinsics.ppx,
                                 observed_z / self.camera.intrinsics.fx)
        observed_y = np.multiply(np.squeeze(self.observed_pix[:, [1]]) - self.camera.intrinsics.ppy,
                                 observed_z / self.camera.intrinsics.fy)

        # Combine the scaled points into a new observed point set
        new_observed_pts = np.asarray([observed_x, observed_y, observed_z]).T

        # Compute the rigid transform between the new observed points and the measured points
        R, t = self._get_rigid_transform(np.asarray(new_observed_pts), np.asarray(self.measured_pts))
        t.shape = (3, 1)
        self.camera2world = np.concatenate((np.concatenate((R, t), axis=1), np.array([[0, 0, 0, 1]])), axis=0)

        # Compute the error between the registered points and the measured points
        registered_pts = np.dot(R, np.transpose(new_observed_pts)) + np.tile(t, (1, new_observed_pts.shape[0]))
        error = np.transpose(registered_pts) - self.measured_pts
        error = np.sum(np.multiply(error, error))
        rmse = np.sqrt(error / new_observed_pts.shape[0])
        return rmse

    def _generate_grid(self):
        """
        Generate a 3D calibration grid within the workspace limits.

        :return: Calibration grid points as a NumPy array.
        """
        # Generate grid points along each axis
        gridspace_x = np.linspace(self.workspace_limits[0][0], self.workspace_limits[0][1],
                                  1 + (self.workspace_limits[0][1] - self.workspace_limits[0][0]) / self.calib_grid_step)
        gridspace_y = np.linspace(self.workspace_limits[1][0], self.workspace_limits[1][1],
                                  1 + (self.workspace_limits[1][1] - self.workspace_limits[1][0]) / self.calib_grid_step)
        gridspace_z = np.linspace(self.workspace_limits[2][0], self.workspace_limits[2][1],
                                  1 + (self.workspace_limits[2][1] - self.workspace_limits[2][0]) / self.calib_grid_step)

        # Create a meshgrid of the points
        calib_grid_x, calib_grid_y, calib_grid_z = np.meshgrid(gridspace_x, gridspace_y, gridspace_z)

        # Flatten the grid points into a list of 3D points
        num_calib_grid_pts = calib_grid_x.shape[0] * calib_grid_x.shape[1] * calib_grid_x.shape[2]
        calib_grid_x.shape = (num_calib_grid_pts, 1)
        calib_grid_y.shape = (num_calib_grid_pts, 1)
        calib_grid_z.shape = (num_calib_grid_pts, 1)
        calib_grid_pts = np.concatenate((calib_grid_x, calib_grid_y, calib_grid_z), axis=1)
        return calib_grid_pts

    def run(self):
        """
        Perform the calibration process by collecting data and optimizing the camera parameters.
        """
        # Connect to the camera
        self.camera.connect()
        logging.debug(self.camera.intrinsics)

        logging.info('Collecting data...')

        # Generate the calibration grid
        calib_grid_pts = self._generate_grid()
        logging.info('Total grid points: %d', calib_grid_pts.shape[0])

        # Iterate through each grid point and collect data
        for tool_position in calib_grid_pts:
            logging.info('Requesting move to tool position: %s', tool_position)
            np.save(self.tool_position, tool_position)
            np.save(self.move_completed, 0)

            # Wait for the robot to complete the move
            while not np.load(self.move_completed):
                time.sleep(0.1)
            time.sleep(2)  # Allow the robot to stabilize

            # Capture images and find the checkerboard
            image_bundle = self.camera.get_image_bundle()
            camera_color_img = image_bundle['rgb']
            camera_depth_img = image_bundle['aligned_depth']
            bgr_color_data = cv2.cvtColor(camera_color_img, cv2.COLOR_RGB2BGR)
            gray_data = cv2.cvtColor(bgr_color_data, cv2.COLOR_RGB2GRAY)
            checkerboard_found, corners = cv2.findChessboardCorners(gray_data, (3, 3), None, cv2.CALIB_CB_ADAPTIVE_THRESH)

            if checkerboard_found:
                # Refine the checkerboard corners
                refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners_refined = cv2.cornerSubPix(gray_data, corners, (3, 3), (-1, -1), refine_criteria)

                # Compute the 3D position of the checkerboard center
                checkerboard_pix = np.round(corners_refined[4, 0, :]).astype(int)
                checkerboard_z = camera_depth_img[checkerboard_pix[1]][checkerboard_pix[0]]
                if checkerboard_z == 0:
                    continue
                checkerboard_x = np.multiply(checkerboard_pix[0] - self.camera.intrinsics.ppx,
                                             checkerboard_z / self.camera.intrinsics.fx)
                checkerboard_y = np.multiply(checkerboard_pix[1] - self.camera.intrinsics.ppy,
                                             checkerboard_z / self.camera.intrinsics.fy)

                # Save the observed and measured points
                self.observed_pts.append([checkerboard_x, checkerboard_y, checkerboard_z])
                tool_position = tool_position + self.checkerboard_offset_from_tool
                self.measured_pts.append(tool_position)
                self.observed_pix.append(checkerboard_pix)

                # Visualize the checkerboard
                vis = cv2.drawChessboardCorners(bgr_color_data, (1, 1), corners_refined[4, :, :], checkerboard_found)
                cv2.imshow('Calibration', vis)
                cv2.waitKey(10)
            else:
                logging.info('Checkerboard not found')

        # Convert collected data to NumPy arrays
        self.measured_pts = np.asarray(self.measured_pts)
        self.observed_pts = np.asarray(self.observed_pts)
        self.observed_pix = np.asarray(self.observed_pix)

        # Optimize the depth scale to minimize the rigid transform error
        logging.info('Calibrating...')
        z_scale_init = 1
        optim_result = optimize.minimize(self._get_rigid_transform_error, np.asarray(z_scale_init), method='Nelder-Mead')
        camera_depth_offset = optim_result.x

        # Save the optimized depth scale and camera pose
        logging.info('Saving...')
        np.savetxt('saved_data/camera_depth_scale.txt', camera_depth_offset, delimiter=' ')
        rmse = self._get_rigid_transform_error(camera_depth_offset)
        logging.info('RMSE: %f', rmse)
        np.savetxt('saved_data/camera_pose.txt', self.camera2world, delimiter=' ')
        logging.info('Done.')
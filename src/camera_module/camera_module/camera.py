import logging
import matplotlib.pyplot as plt
import numpy as np
import pyrealsense2 as rs

# Set up logging for the script
logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)  # Set logging level to INFO


class RealSenseCamera:
    """
    A class to interface with an Intel RealSense camera.
    Provides methods to connect to the camera, retrieve image frames, and visualize them.
    """
    def __init__(self,
                 device_id,
                 width=640,
                 height=480,
                 fps=30):
        """
        Initialize the RealSenseCamera object.

        :param device_id: Serial number of the camera to connect to.
        :param width: Width of the image stream (default: 640).
        :param height: Height of the image stream (default: 480).
        :param fps: Frames per second for the stream (default: 30).
        """
        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps

        # Initialize placeholders for the camera pipeline, depth scale, and intrinsics
        self.pipeline = None
        self.scale = None
        self.intrinsics = None

    def connect(self):
        """
        Connect to the RealSense camera and configure the streams.
        Sets up depth and color streams and retrieves camera intrinsics and depth scale.
        """
        try:
            # Create a pipeline to manage the camera streams
            self.pipeline = rs.pipeline()
            config = rs.config()

            # Enable the device using its serial number
            config.enable_device(str(self.device_id))

            # Enable depth and color streams with specified resolution and FPS
            config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps)

            # Start the pipeline with the configured streams
            cfg = self.pipeline.start(config)

            # Retrieve the intrinsics of the color stream
            rgb_profile = cfg.get_stream(rs.stream.color)
            self.intrinsics = rgb_profile.as_video_stream_profile().get_intrinsics()

            # Retrieve the depth scale (used to convert depth values to meters)
            self.scale = cfg.get_device().first_depth_sensor().get_depth_scale()

            # Log successful connection and camera details
            logging.info("Camera connected successfully.")
            logging.info(f"Device ID: {self.device_id}")
            logging.info(f"Depth scale: {self.scale}")
            logging.info(f"Camera intrinsics: {self.intrinsics}")
        except Exception as e:
            # Log any errors that occur during connection
            logging.error(f"Failed to connect to camera: {e}")
            raise

    def get_image_bundle(self):
        """
        Retrieve a bundle of aligned depth and color images from the camera.

        :return: A dictionary containing 'rgb' (color image) and 'aligned_depth' (depth image).
        """
        # Wait for a new set of frames from the camera
        frames = self.pipeline.wait_for_frames()

        # Align the depth frame to the color frame
        align = rs.align(rs.stream.color)
        aligned_frames = align.process(frames)

        # Retrieve the aligned color and depth frames
        color_frame = aligned_frames.first(rs.stream.color)
        aligned_depth_frame = aligned_frames.get_depth_frame()

        # Check if the frames are valid
        if not aligned_depth_frame or not color_frame:
            logging.error("Failed to retrieve frames from the camera.")
            return None

        # Convert the depth frame to a NumPy array and scale it to meters
        depth_image = np.asarray(aligned_depth_frame.get_data(), dtype=np.float32)
        depth_image *= self.scale

        # Convert the color frame to a NumPy array
        color_image = np.asanyarray(color_frame.get_data())

        # Expand the depth image dimensions to match the color image (for visualization)
        depth_image = np.expand_dims(depth_image, axis=2)

        # Return the images as a dictionary
        return {
            'rgb': color_image,
            'aligned_depth': depth_image,
        }

    def plot_image_bundle(self):
        """
        Plot the retrieved color and depth images side by side.
        """
        # Retrieve the image bundle
        images = self.get_image_bundle()
        if images is None:
            logging.error("No images to plot.")
            return

        # Extract the color and depth images
        rgb = images['rgb']
        depth = images['aligned_depth']

        # Create a figure with two subplots
        fig, ax = plt.subplots(1, 2, squeeze=False)

        # Display the color image
        ax[0, 0].imshow(rgb)

        # Compute the mean and standard deviation of the depth image for normalization
        m, s = np.nanmean(depth), np.nanstd(depth)

        # Display the depth image with a grayscale colormap
        ax[0, 1].imshow(depth.squeeze(axis=2), vmin=m - s, vmax=m + s, cmap=plt.cm.gray)

        # Set titles for the subplots
        ax[0, 0].set_title('rgb')
        ax[0, 1].set_title('aligned_depth')

        # Show the plots
        plt.show()


if __name__ == '__main__':
    """
    Main script to connect to the camera and continuously display the image bundle.
    """
    # Create a RealSenseCamera object with the specified device ID
    cam = RealSenseCamera(device_id="337122071438")  # Ensure device_id is a string

    try:
        # Connect to the camera
        cam.connect()

        # Continuously retrieve and plot images
        while True:
            cam.plot_image_bundle()
    except Exception as e:
        # Log any errors that occur during execution
        logging.error(f"An error occurred: {e}")
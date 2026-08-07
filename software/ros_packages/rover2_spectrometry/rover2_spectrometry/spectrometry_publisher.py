from cv_bridge import CvBridge
import matplotlib
matplotlib.use("Agg")
# from image_transport_py import ImageTransport
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
import cv2
import numpy as np
import matplotlib.pyplot as plt
from rover2_spectrometry_interface.srv import SpectrometryInterface


# IMPORTANT NOTE: THIS CODE MUST BE UPDATED TO ACCOMODATE THE ACTUAL DESIGN
# If this is deployed onto the rover, change this value to True
IMPLEMENTATION_VERSION = False

class Spectrometry_Publisher(Node):
    """
    On service call, starts a feed reading science data from Benedict's and Ninhydrin reactions.
    Gets image, puts it on a chart showing average RGB values per pixel on the x-axis.
    Publishes all in a batch every 10 seconds from startup of node.
    """
    def __init__(self):
        super().__init__('Spectrometry_Publisher')

        self.img_pub = self.create_publisher(Image, 'science/image', 10)

        self.bridge = CvBridge()
        # [Top_Left_X, Top_Left_Y, Width, Height]
        # The following are dummy inputs
        self.x_start = 2
        self.x_size = 800
        self.y_start = 2
        self.y_size = 800
        self.r = [self.x_start, self.y_start, self.x_size, self.y_size]
        # The following is for old scimech:
        # self.r = [286, 60, 27, 54]
        self.start_time = 0
        self.camera_ids = []

        self.declare_parameter("camera_location", "/dev/video")

        self.timer_period = .1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.srv = self.create_service(SpectrometryInterface, 'spectrometry_chart', self.start_graph_feed)

        self.reaction_type = {}
        self.start_times = {}
        self.iterations = {}
        self._camera_location = self.get_parameter("camera_location").value

    def start_graph_feed(self, request, response):
        if not self.camera_ids:
            self.start_time = self.get_clock().now()
        try:
            if(request.camera_id in self.camera_ids):
                response.success = False
                return response
            self.camera_ids.append(request.camera_id)
            self.reaction_type.update({request.camera_id: request.reaction_type})
            self.start_times.update({request.camera_id: self.get_clock().now()})
            self.iterations.update({request.camera_id: 0    })
            response.success = True
            return response
        except:
            response.success = False
            return response



    def timer_callback(self):
        # Check if any camera ids have been requested
        if not self.camera_ids:
            return
        for camera_id in self.camera_ids:
            if ((int(self.start_times.get(camera_id).nanoseconds / 1e9) + (10.0 * self.iterations.get(camera_id)))
                    > float(self.get_clock().now().nanoseconds)/1e9):
                continue
            self.iterations[camera_id] = self.iterations[camera_id] + 1
            rgb = self.get_graph(camera_id, self.reaction_type.get(camera_id))


            # matplotlib gives RGB, OpenCV expects BGR
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            image_msg = self.bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera_' + str(camera_id)

            self.img_pub.publish(image_msg)
            reaction_type = self.reaction_type.get(camera_id)
            if reaction_type == 1:
                title = "Benedict's Control"
            elif reaction_type == 2:
                title = "Benedict's Experiment"
            elif reaction_type == 3:
                title = "Ninhydrin Control"
            elif reaction_type == 4:
                title = "Ninhydrin Experiment"
            elif reaction_type == 5:
                title = "Dummy Image"
            else:
                raise Exception("Unknown reaction type: reaction type must be an integer between 1 and 4")
            self.get_logger().info('Publishing image from camera ' + str(camera_id) + ' and reaction type ' + title + ' and time=')
            plt.clf()

    def get_graph(self, feedNumber, reaction_type):
        cap = cv2.VideoCapture(self._camera_location + str(feedNumber))
        
        ret, frame = cap.read()
        cropped = frame[int(self.r[1]):int(self.r[1] + self.r[3]), int(self.r[0]):int(self.r[0] + self.r[2])]
        # cv2.imshow('roi', cropped)
        shape = cropped.shape

        r_dist = []
        b_dist = []
        g_dist = []
        i_dist = []
        for i in range(shape[1]):
            r_val = np.mean(cropped[:, i][:, 2])
            b_val = np.mean(cropped[:, i][:, 0])
            g_val = np.mean(cropped[:, i][:, 1])
            i_val = (r_val + b_val + g_val) / 3

            r_dist.append(r_val)
            g_dist.append(g_val)
            b_dist.append(b_val)
            i_dist.append(i_val)

        figs, axs = plt.subplots(2, 1, sharex=True)

        axs[0].imshow(cv2.cvtColor(frame[int(self.r[1]):int(self.r[1] + self.r[3]), int(self.r[0]):int(self.r[0] + self.r[2])], cv2.COLOR_BGR2RGB),
                      aspect="auto")
        axs[0].axis('off')

        axs[1].plot(r_dist, color='r', label='red')
        axs[1].plot(g_dist, color='g', label='green')
        axs[1].plot(b_dist, color='b', label='blue')
        axs[1].plot(i_dist, color='k', label='mean')
        axs[1].legend(loc="upper left")

        axs[1].set_xlabel("Image X-Axis")
        axs[1].set_ylabel("Average Intensity")

        now = self.get_clock().now()
        elapsed = int((now - self.start_times.get(feedNumber)).nanoseconds / 1e9)
        print("Chart created for " + str(feedNumber) + " at " +
              str((now - self.start_times.get(feedNumber)).nanoseconds / 1e9))

        plt.legend(loc="upper left")
        # fig = plt.gcf()

        # This is where we would implement the different experiment titles.

        # Titles:
        # 1) Benedicts Control
        # 2) Benedicts Experiment
        # 3) Ninhydrin Control
        # 4) Ninhydrin Experiment

        # This is to ensure compatibility with other commands that are floating around.
        # 1 and 2 mean benedicts, 3 and 4 mean ninhydrin.
        # 5 is a default value that shouldn't get used.
        if (reaction_type == 1 or reaction_type == 2) and elapsed < 5:
            title = "Benedict's Control"
        elif (reaction_type == 1 or reaction_type == 2) and elapsed > 5:
            title = "Benedict's Experiment"
        elif (reaction_type == 3 or reaction_type == 4) and elapsed < 5:
            title = "Ninhydrin Control"
        elif (reaction_type == 3 or reaction_type == 4) and elapsed > 5:
            title = "Ninhydrin Experiment"
        elif reaction_type == 5:
            title = "Spectroanalysis of Image"
        else:
            raise Exception("Unknown reaction type: reaction type must be an integer between 1 and 4")



        # plt.suptitle(f"Average Intensity by X-Axis | Time: {elapsed}s", fontsize=16)
        plt.suptitle(f"{title} | Time: {elapsed}s", fontsize=16)

        rgb = Spectrometry_Publisher.fig_to_numpy(figs)
        return rgb



    @staticmethod
    def fig_to_numpy(fig):
        fig.canvas.draw()
        
        # Get the image as ARGB byte buffer
        buf = np.frombuffer(fig.canvas.tostring_argb(), dtype=np.uint8)

        w, h = fig.canvas.get_width_height()
        buf = buf.reshape((h, w, 4))

        # Convert ARGB → RGB
        rgb = buf[:, :, 1:4]
        return rgb


def main(args=None):
    rclpy.init(args=args)
    node = Spectrometry_Publisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly (optional)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

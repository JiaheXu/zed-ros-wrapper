########################################################################
#
# Copyright (c) 2022, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################
# zed sdk
import pyzed.sl as sl

# ros stuff
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
bridge = CvBridge()
import std_msgs

class zed_cam():

    def __init__(self, left_cam_topic = "/zed2/left/image_raw", right_cam_topic="/zed2/right/image_raw", depth_output_topic="/zed2/depth", depth_output = False , output_hz = 30 ):
        # Create a Camera object
        self.zed = sl.Camera()

        # Create a InitParameters object and set configuration parameters
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.AUTO # Use HD720 opr HD1200 video mode, depending on camera type.
        self.init_params.camera_fps = output_hz  # Set fps at 30 

        self.left_img_pub = rospy.Publisher(left_cam_topic, Image, queue_size=100)
        self.right_img_pub = rospy.Publisher(right_cam_topic, Image, queue_size=100)

        image_left = sl.Mat()
        image_right = sl.Mat()
        runtime_parameters = sl.RuntimeParameters()
        
        # Open the camera
        err = self.zed.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print("Camera Open : "+repr(err)+". Exit program.")
            exit()

        while not rospy.is_shutdown():
            # Grab an image, a RuntimeParameters object must be given to grab()
            if self.zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                # A new image is available if grab() returns SUCCESS
                self.zed.retrieve_image(image_left, sl.VIEW.LEFT)
                self.zed.retrieve_image(image_right, sl.VIEW.RIGHT)
                # print("image_left: ", type(image_left) )
                
                left_bgra = image_left.get_data()
                right_bgra = image_right.get_data()

                header = std_msgs.msg.Header()
                header.stamp = rospy.Time.now()

                rgb_left = cv2.cvtColor(left_bgra, cv2.COLOR_BGRA2RGB)
                rgb_right = cv2.cvtColor(right_bgra, cv2.COLOR_BGRA2RGB)

                left_img_msg = bridge.cv2_to_imgmsg(rgb_left, encoding="rgb8")
                left_img_msg.header = header
                self.left_img_pub.publish(left_img_msg)                

                right_img_msg = bridge.cv2_to_imgmsg(rgb_right, encoding="rgb8")
                right_img_msg.header = header
                self.right_img_pub.publish(right_img_msg)
                # timestamp = self.zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)  # Get the timestamp at the time the image was captured
                # print("Image resolution: {0} x {1} || Image timestamp: {2}\n".format(image_left.get_width(), image_left.get_height(),
                    # timestamp.get_milliseconds()))
                # i = i + 1

        self.zed.close()


    def run(self):
        rospy.spin()


def main():

    rospy.init_node("zed2_node")
    zed_cam_node = zed_cam()
    # zed_cam_node.run()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# Copyright 2021 Seek Thermal Inc.
#
# Original author: Michael S. Mead <mmead@thermal.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import cv2, rospy

from threading import Condition
from seekcamera import (
    SeekCameraIOType,
    SeekCameraColorPalette,
    SeekCameraManager,
    SeekCameraManagerEvent,
    SeekCameraFrameFormat,
    SeekCamera,
    SeekCameraFrameHeader,
    SeekFrame
)

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from seekcamera_ros.msg import CameraFrame
from cv_bridge import CvBridge, CvBridgeError

class ThermalCamera:
    def __init__(self):
        rospy.init_node("thermal_camera", anonymous=False)
        
        self.cvBridge = CvBridge()
        self.window_name = "Seek Thermal - Python OpenCV Sample"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        rospy.Subscriber("/thermal_camera/image_raw", Image, callback=self.showImageFromMsg, queue_size=10)
        rospy.Subscriber("/thermal_camera/info", CameraFrame, callback=self.showInfoFromMsg, queue_size=10)
        

        self.main()

    def showImageFromMsg(self, msg):
        self.cvBridge.imgmsg_to_cv2()
        cv2.imshow(self.window_name, img)



    def showInfoFromMsg(self, msg):
        pass

    def main(self):

        # Create a context structure responsible for managing all connected USB cameras.
        # Cameras with other IO types can be managed by using a bitwise or of the
        # SeekCameraIOType enum cases.
        while not rospy.is_shutdown():
            # Wait a maximum of 150ms for each frame to be received.
            # A condition variable is used to synchronize the access to the renderer;
            # it will be notified by the user defined frame available callback thread.
                if renderer.frame_condition.wait(150.0e-3):
                    img = renderer.frame.data
                    image_msg = self.cvBridge.cv2_to_imgmsg(img)
                    self.raw_image_publisher.publish(image_msg)

                    # Resize the rendering window.
                    if renderer.first_frame:
                        (height, width, _) = img.shape
                        cv2.resizeWindow(window_name, width * 2, height * 2)
                        renderer.first_frame = False

                    # Render the image to the window.

            # Process key events.
            key = cv2.waitKey(1)
            if key == ord("q"):
                break

            # Check if the window has been closed manually.
            if not cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE):
                break

        cv2.destroyWindow(window_name)

if __name__ == '__main__':
    try:
        ThermalCamera()
    except rospy.ROSInterruptException:
        pass

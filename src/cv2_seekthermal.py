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

from numpy import ndarray

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from seekcamera_ros.msg import CameraFrame
from cv_bridge import CvBridge, CvBridgeError

class ThermalCamera:
    def __init__(self):
        rospy.init_node("thermal_camera_visualizer", anonymous=False)
        
        self.cvBridge = CvBridge()
        self.img = None

        rospy.Subscriber("/thermal_camera/image_raw", Image, callback=self.showImageFromMsg, queue_size=10)
        rospy.Subscriber("/thermal_camera/info", CameraFrame, callback=self.showInfoFromMsg, queue_size=10)
        

        window_name = "Seek Thermal - Python OpenCV Sample"
        first_frame = True
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        while not rospy.is_shutdown():
            # Wait a maximum of 150ms for each frame to be received.
            # A condition variable is used to synchronize the access to the renderer;
            # it will be notified by the user defined frame available callback thread.
            # Process key events.


            while self.img is None:
                print(type(self.img))
                rospy.sleep(0.1)
                pass
            if first_frame:
                (height, width, _) = self.img.shape
                cv2.resizeWindow(window_name, width * 2, height * 2)
                first_frame = False
            # Render the image to the window.
            cv2.imshow(window_name, self.img)
            key = cv2.waitKey(1)
            if key == ord("q"):
                break
            # Check if the window has been closed manually.
            if not cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE):
                break
            rospy.sleep(0.05)
        cv2.destroyWindow(window_name)

    def showImageFromMsg(self, msg):
        self.img = self.cvBridge.imgmsg_to_cv2(img_msg=msg)

    def showInfoFromMsg(self, msg):
        pass

if __name__ == '__main__':
    try:
        ThermalCamera()
    except rospy.ROSInterruptException:
        pass

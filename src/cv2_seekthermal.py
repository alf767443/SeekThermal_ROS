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

import cv2, rospy, numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from seekcamera_ros.msg import CameraFrame as CameraFrameMsg, SeekCamera as SeekCameraMsg
from cv_bridge import CvBridge, CvBridgeError

class ThermalCamera:
    def __init__(self):
        rospy.init_node("thermal_camera_visualizer", anonymous=False)
        
        self.cvBridge = CvBridge()
        self.frame = None
        self.center_target = None

        rospy.Subscriber("/thermal_camera/image_raw", Image, callback=self.showImageFromMsg, queue_size=10)
        rospy.Subscriber("/thermal_camera/info/frame", CameraFrameMsg, callback=self.showInfoFrameFromMsg, queue_size=10)
        rospy.Subscriber("/thermal_camera/info/camera", SeekCameraMsg, callback=self.showInfoCameraFromMsg, queue_size=10)

        

        window_name = "Seek Thermal - Python OpenCV Sample"
        first_frame = True
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        print(f"Press:\n\tq: quit\n\tc: capture")

        while not rospy.is_shutdown():
            # Wait a maximum of 150ms for each frame to be received.
            # A condition variable is used to synchronize the access to the renderer;
            # it will be notified by the user defined frame available callback thread.
            # Process key events.
            while self.frame is None:
                print(type(self.frame))
                rospy.sleep(0.1)
                pass
            if first_frame:
                (height, width, _) = self.frame.shape
                cv2.resizeWindow(window_name, width * 2, height * 2)
                first_frame = False
            image = self.frame            
            if not self.center_target is None:
                image = cv2.addWeighted(image, 1, self.center_target, 1, 0)

            cv2.imshow(window_name, image)
            key = cv2.waitKey(1)
            if key == ord("q"):
                break
            # Check if the window has been closed manually.
            if not cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE):
                break
            rospy.sleep(0.05)
        cv2.destroyWindow(window_name)

    def showImageFromMsg(self, msg):
        self.frame = self.cvBridge.imgmsg_to_cv2(img_msg=msg)

    def showInfoFrameFromMsg(self, msg):
        if not self.frame is None:
            self.center_target = self.targetOverImage(msg.thermography.spot.x, msg.thermography.spot.y, msg.thermography.spot.temperature, 0.03, (0, 0, 255), self.frame)            

    def showInfoCameraFromMsg(self, msg):
        pass

    def targetOverImage(self, x, y, temperature, size, color,  frame):
        (height, width, _) = frame.shape
        mask = np.zeros_like(frame)
        cv2.circle(mask, (x, y), int(height*size), color, int(height*0.01))
        cv2.line(mask,(x-int(height*size), y),(x+int(height*size), y),color,int(height*0.01))
        cv2.line(mask,(x, y-int(height*size)),(x, y+int(height*size)),color,int(height*0.01))
        text = self.textOverImage(x+int(height*size), y-int(height*size), f"{temperature:.2f}", color, height*size*0.2, 1, frame)
        mask = cv2.addWeighted(mask, 1, text, 1, 0)
        return mask

    def textOverImage(self, x, y, text, color, size, thick, frame):
        mask = np.zeros_like(frame)
        fonte = cv2.FONT_HERSHEY_PLAIN  
        position = (x, y) 
        cv2.putText(mask, text, position, fonte, size, color, thick)
        return mask


if __name__ == '__main__':
    try:
        ThermalCamera()
    except rospy.ROSInterruptException:
        pass

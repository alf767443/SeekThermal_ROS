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

import rospy
from threading import Condition
from seekcamera import (
    SeekCameraIOType,
    SeekCameraColorPalette,
    SeekCameraManager,
    SeekCameraManagerEvent,
    SeekCameraFrameFormat,
    SeekCameraAGCMode,
    SeekCameraShutterMode,
    SeekCameraFrameHeader,
    SeekCameraTemperatureUnit,
    SeekCamera,
    SeekFrame
)
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from seekcamera_ros.msg import CameraFrame
from cv_bridge import CvBridge, CvBridgeError

class Renderer:
    """Contains camera and image data required to render images to the screen."""
    def __init__(self):
        self.busy = False
        self.frame = SeekFrame()
        self.camera = SeekCamera()
        self.frame_condition = Condition()
        self.first_frame = True

class ThermalCamera:
    def __init__(self):
        rospy.init_node("thermal_camera", anonymous=False)
        # Global variables
        self.CameraParameters = None
        # Init publishers and the CvBridge
        self.raw_image_publisher = rospy.Publisher("/image_raw", Image, queue_size=10)
        self.raw_info_publisher = rospy.Publisher("/info", CameraFrame, queue_size=10)
        self.cvBridge = CvBridge()
        # Get the parameters
        self.getParameters()
        # Run the main function
        self.main()
    
    def on_frame(self, _camera: SeekCamera, camera_frame: Condition, renderer: Renderer):
        """Async callback fired whenever a new frame is available.

        Parameters
        ----------
        _camera: SeekCamera
            Reference to the camera for which the new frame is available.
        camera_frame: SeekCameraFrame
            Reference to the class encapsulating the new frame (potentially
            in multiple formats).
        renderer: Renderer
            User defined data passed to the callback. This can be anything
            but in this case it is a reference to the renderer object.
        """

        # Acquire the condition variable and notify the main thread
        # that a new frame is ready to render. This is required since
        # all rendering done by OpenCV needs to happen on the main thread.
        with renderer.frame_condition:
            renderer.frame = camera_frame.color_argb8888
            renderer.frame_condition.notify()

    def on_event(self, camera: SeekCamera, event_type: SeekCameraManagerEvent, event_status, renderer: Renderer):
        """Async callback fired whenever a camera event occurs.

        Parameters
        ----------
        camera: SeekCamera
            Reference to the camera on which an event occurred.
        event_type: SeekCameraManagerEvent
            Enumerated type indicating the type of event that occurred.
        event_status: Optional[SeekCameraError]
            Optional exception type. It will be a non-None derived instance of
            SeekCameraError if the event_type is SeekCameraManagerEvent.ERROR.
        renderer: Renderer
            User defined data passed to the callback. This can be anything
            but in this case it is a reference to the Renderer object.
        """
        print("{}: {}".format(str(event_type), camera.chipid))
        if event_type == SeekCameraManagerEvent.CONNECT:
            if renderer.busy:
                return
            # Claim the renderer.
            # This is required in case of multiple cameras.
            renderer.busy = True
            renderer.camera = camera
            # Indicate the first frame has not come in yet.
            # This is required to properly resize the rendering window.
            renderer.first_frame = True
            # Set a custom color palette.
            # Other options can set in a similar fashion.
            camera.color_palette = SeekCameraColorPalette.TYRIAN
            # Start imaging and provide a custom callback to be called
            # every time a new frame is received.
            camera.register_frame_available_callback(self.on_frame, renderer)
            camera.capture_session_start(SeekCameraFrameFormat.COLOR_ARGB8888)
        elif event_type == SeekCameraManagerEvent.DISCONNECT:
            # Check that the camera disconnecting is one actually associated with
            # the renderer. This is required in case of multiple cameras.
            if renderer.camera == camera:
                # Stop imaging and reset all the renderer state.
                camera.capture_session_stop()
                renderer.camera = None
                renderer.frame = None
                renderer.busy = False
        elif event_type == SeekCameraManagerEvent.ERROR:
            print("{}: {}".format(str(event_status), camera.chipid))
        elif event_type == SeekCameraManagerEvent.READY_TO_PAIR:
            return

    def main(self):
        # Create a context structure responsible for managing all connected USB cameras.
        # Cameras with other IO types can be managed by using a bitwise or of the
        # SeekCameraIOType enum cases.
        with SeekCameraManager(SeekCameraIOType.USB ) as manager:
            # Start listening for events.
            renderer = Renderer()
            manager.register_event_callback(self.on_event, renderer)

            while not rospy.is_shutdown():
                # Wait a maximum of 150ms for each frame to be received.
                # A condition variable is used to synchronize the access to the renderer;
                # it will be notified by the user defined frame available callback thread.
                with renderer.frame_condition:
                    if not renderer.frame.header is None:
                        frame= renderer.frame.header
                        frame_msg = self.SeekFrame2msg(frame)
                        self.raw_info_publisher.publish(frame_msg)
                    if renderer.frame_condition.wait(150.0e-3):
                        img = renderer.frame.data
                        image_msg = self.cvBridge.cv2_to_imgmsg(img)
                        self.raw_image_publisher.publish(image_msg)

    # 
    def SeekFrame2msg(self, frameHeader: SeekCameraFrameHeader)->CameraFrame:
        try:
            # Begin message
            header = Header()
            header.stamp = rospy.Time.now()
            msg = CameraFrame()
            msg.header                  = header            
            msg.sentinel                = frameHeader.sentinel
            msg.version                 = frameHeader.version
            msg.frame.width             = frameHeader.width
            msg.frame.height            = frameHeader.height
            msg.frame.channels          = frameHeader.channels
            msg.pixel.depth             = frameHeader.pixel_depth
            msg.pixel.padding           = frameHeader.pixel_padding
            msg.camera.chip_id          = frameHeader.chipid
            msg.camera.serial_number    = frameHeader.serial_number
            # msg.camera.core_part_number = frameHeader.core_part_number
            msg.fpa.frame_count         = frameHeader.fpa_frame_count
            msg.fpa.diode_count         = frameHeader.fpa_diode_count
            msg.thermography.environment_temperature = frameHeader.environment_temperature
            (msg.thermography.min.x, msg.thermography.min.y, msg.thermography.min.temperature)    = frameHeader.thermography_min
            (msg.thermography.max.x, msg.thermography.max.y, msg.thermography.max.temperature)    = frameHeader.thermography_max
            (msg.thermography.spot.x, msg.thermography.spot.y, msg.thermography.spot.temperature) = frameHeader.thermography_spot
            return msg
        except Exception as e:
            rospy.logerr(f"An error occurred when converting SeekFrame to rosmsg")

    def getParameters(self)->bool:
        try:
            # Get the camera parameters from ROS parameters
            CameraParameters = {
                'thermography_window': rospy.get_param('thermal_camera/thermography_window', (0,0,100,100)),
                'thermography_offset': rospy.get_param('thermal_camera/thermography_offset', 0),
                'temperature_unit'   : SeekCameraTemperatureUnit(rospy.get_param('thermal_camera/temperature_unit', 0)),
                'color_palette'      : SeekCameraColorPalette(rospy.get_param('thermal_camera/color_palette', 0)),
                'shutter_mode'       : SeekCameraShutterMode(rospy.get_param('thermal_camera/color_palette', 0)),
                'agc_mode'           : SeekCameraAGCMode(rospy.get_param('thermal_camera/color_palette', 0))
            }
            # Parameters are change/new
            if self.CameraParameters is None:
                rospy.loginfo(f"Initialising the camera parameters:\n{CameraParameters}")
                self.CameraParameters = CameraParameters
                return True
            elif not self.CameraParameters == CameraParameters:
                rospy.loginfo(f"Camera parameters are changed... New parameters:\n{CameraParameters}")
                self.CameraParameters = CameraParameters
                return True
            # No changes
            else:
                return False
        except Exception as e:
            rospy.logerr(f"An error occurred when obtaining the camera parameters\n{e}")
            return False
    
    def setCameraParametres(self, camera: SeekCamera)->bool:
        camera.thermography_window = self.CameraParameters['thermography_window']
        camera.thermography_offset = self.CameraParameters['thermography_offset']
        camera.temperature_unit    = self.CameraParameters['temperature_unit']
        camera.color_palette       = self.CameraParameters['color_palette']
        camera.agc_mode            = self.CameraParameters['agc_mode']
        pass

if __name__ == '__main__':
    try:
        ThermalCamera()
    except rospy.ROSInterruptException:
        pass

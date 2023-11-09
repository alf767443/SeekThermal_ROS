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
from seekcamera_ros.msg import CameraFrame as CameraFrameMsg, SeekCamera as SeekCameraMsg
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
        self.raw_image_publisher = rospy.Publisher("thermal_camera/image_raw", Image, queue_size=10)
        self.info_frame_publisher = rospy.Publisher("thermal_camera/info/frame", CameraFrameMsg, queue_size=10)
        self.info_camera_publisher = rospy.Publisher("thermal_camera/info/camera", SeekCameraMsg, queue_size=10)
        self.cvBridge = CvBridge()
        # Get the parameters
        self.getParameters()
        # Open and monitor the camera
        with SeekCameraManager(SeekCameraIOType.USB ) as manager:
            # Start listening for events.
            renderer = Renderer()
            manager.register_event_callback(self.on_event, renderer)
            while not rospy.is_shutdown():
                with renderer.frame_condition:
                    pass
    
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
            # Publish the frame in /info
            frame= renderer.frame.header
            frame_msg = self.SeekFrame2msg(frame)
            self.info_frame_publisher.publish(frame_msg)
            # Publish the image in /image
            img = renderer.frame.data
            image_msg = self.cvBridge.cv2_to_imgmsg(img)
            self.raw_image_publisher.publish(image_msg)
            # Publish the camara info
            camera = _camera
            camera_msg = self.SeekCamera2msg(camera)
            self.info_camera_publisher.publish(camera_msg)

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
        if event_type == SeekCameraManagerEvent.CONNECT:
            rospy.loginfo(f"Camera connect: {camera.chipid}")
            if renderer.busy:
                return
            # Claim the renderer.
            # This is required in case of multiple cameras.
            renderer.busy = True
            renderer.camera = camera
            # Indicate the first frame has not come in yet.
            renderer.first_frame = True
            # Set camera parameters
            camera = self.setCameraParametres(camera=camera)

            camera.register_frame_available_callback(self.on_frame, renderer)
            camera.capture_session_start(SeekCameraFrameFormat.COLOR_ARGB8888)
        elif event_type == SeekCameraManagerEvent.DISCONNECT:
            # Check that the camera disconnecting is one actually associated with
            # the renderer. This is required in case of multiple cameras.
            rospy.loginfo(f"Camera disconnect: {camera.chipid}")
            if renderer.camera == camera:
                # Stop imaging and reset all the renderer state.
                camera.capture_session_stop()
                renderer.camera = None
                renderer.frame = None
                renderer.busy = False
        elif event_type == SeekCameraManagerEvent.ERROR:
            rospy.logerr("Camera error\n{}: {}".format(str(event_status), camera.chipid))
        elif event_type == SeekCameraManagerEvent.READY_TO_PAIR:
            return

    def SeekFrame2msg(self, frameHeader: SeekCameraFrameHeader)->CameraFrameMsg:
        try:
            # Begin message
            header = Header()
            header.stamp = rospy.Time.now()
            msg = CameraFrameMsg()
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

    def SeekCamera2msg(self, camera: SeekCamera)->SeekCameraMsg:
        try:
            # Begin message
            header = Header()
            header.stamp = rospy.Time.now()
            msg = SeekCameraMsg()
            msg.header                          = header            
            msg.chipid                          = camera.chipid
            msg.io_type                         = camera.io_type.__str__()
            msg.agc_mode                        = camera.agc_mode.__str__()
            msg.shutter_mode                    = camera.shutter_mode.__str__()
            msg.serial_number                   = camera.serial_number
            msg.color_palette                   = camera.color_palette.__str__()
            msg.temperature_unit                = camera.temperature_unit.__str__()
            msg.firmware_version                = camera.firmware_version.__str__()
            msg.scene_emissivity                = camera.scene_emissivity
            msg.io_properties.type              = camera.io_properties.type.__str__()
            msg.thermography_offset             = camera.thermography_offset
            msg.io_properties.spi.cs_number     = camera.io_properties.spi.cs_number
            msg.io_properties.usb.bus_number    = camera.io_properties.usb.bus_number
            msg.io_properties.spi.bus_number    = camera.io_properties.spi.bus_number
            msg.io_properties.usb.port_numbers  = camera.io_properties.usb.port_numbers
            (msg.thermography_window.x, msg.thermography_window.y, msg.thermography_window.w, msg.thermography_window.h) = camera.thermography_window
            
            return msg
        except Exception as e:
            rospy.logerr(f"An error occurred when converting SeekFrame to rosmsg")

    def getParameters(self)->bool:
        def parameters2str(p:dict)->str:
            _str =  f"\tthermography_offset: {p['thermography_offset']}\n"
            _str += f"\ttemperature_unit   : {p['temperature_unit'].__str__()}\n"
            _str += f"\tcolor_palette      : {p['color_palette'].__str__()}\n"
            _str += f"\tshutter_mode       : {p['shutter_mode'].__str__()}\n"
            _str += f"\tagc_mode           : {p['agc_mode'].__str__()}\n"
            return _str
        try:
            # Get the camera parameters from ROS parameters
            CameraParameters = {
                'thermography_offset': rospy.get_param('thermal_camera/thermography_offset', 0),
                'temperature_unit'   : SeekCameraTemperatureUnit(rospy.get_param('thermal_camera/temperature_unit', 0)),
                'color_palette'      : SeekCameraColorPalette(rospy.get_param('thermal_camera/color_palette', 0)),
                'shutter_mode'       : SeekCameraShutterMode(rospy.get_param('thermal_camera/shutter_mode', 0)),
                'agc_mode'           : SeekCameraAGCMode(rospy.get_param('thermal_camera/agc_mode', 0))
            }
            # Parameters are change/new
            if self.CameraParameters is None:
                rospy.loginfo(f"Initialising the camera parameters:\n{parameters2str(CameraParameters)}")
                self.CameraParameters = CameraParameters
                
                return True
            elif not self.CameraParameters == CameraParameters:
                rospy.loginfo(f"Camera parameters are changed... New parameters:\n{parameters2str(CameraParameters)}")
                self.CameraParameters = CameraParameters
                return True
            # No changes
            else:
                return False
        except Exception as e:
            rospy.logerr(f"An error occurred when obtaining the camera parameters\n{e}")
            return False

    def setCameraParametres(self, camera: SeekCamera)->SeekCamera:
        try:
            rospy.logdebug(f"Setting the paramaters on camera")
            camera.thermography_offset = self.CameraParameters['thermography_offset']
            camera.temperature_unit    = self.CameraParameters['temperature_unit']
            camera.color_palette       = self.CameraParameters['color_palette']            
            camera.shutter_mode        = self.CameraParameters['shutter_mode']
            camera.agc_mode            = self.CameraParameters['agc_mode']
            rospy.loginfo(f"Camera parameters are set")
            return camera
        except Exception as e:
            rospy.logerr(f"An error occurred when setting the camera parameters\n{e}")
            return False
        
if __name__ == '__main__':
    try:
        ThermalCamera()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

import asyncio

import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
# from hardware_test_camera import HardwareTestCamera
from sensor_msgs.msg import CompressedImage as ROSCompressedImage, CameraInfo as ROSCameraInfo

from dt_robot_utils import get_robot_name
from dtps import context
from dtps_http import RawData
from duckietown_messages.sensors.camera import Camera
from duckietown_messages.sensors.compressed_image import CompressedImage
from duckietown_messages.utils.exceptions import DataDecodingError


class CameraNode(DTROS):
    """
    Relays JPEG frames from a DTPS source.

    Publisher:
        ~image/compressed (:obj:`CompressedImage`): The acquired camera images
        ~camera_info (:obj:`CameraInfo`): The camera parameters

    """

    def __init__(self):
        # Initialize the DTROS parent class
        super(CameraNode, self).__init__(
            node_name="camera",
            node_type=NodeType.DRIVER,
            help="Reads a stream of images from a camera and publishes the frames over ROS",
        )
        self._robot_name = get_robot_name()

        # user hardware test
        # self._hardware_test = HardwareTestCamera()

        # Setup publishers
        self._has_published: bool = False
        self.pub_img = rospy.Publisher(
            "~image/compressed",
            ROSCompressedImage,
            queue_size=1,
            dt_topic_type=TopicType.DRIVER,
            dt_help="The stream of JPEG compressed images from the camera",
        )
        self.pub_camera_info = rospy.Publisher(
            "~camera_info",
            ROSCameraInfo,
            latch=True,
            queue_size=1,
            dt_topic_type=TopicType.DRIVER,
            dt_help="The camera calibration information",
        )
        # ---
        self.loginfo("Initialized.")

    async def publish(self, data: RawData):
        # TODO: only publish if somebody is listening
        try:
            jpeg: CompressedImage = CompressedImage.from_rawdata(data)
        except DataDecodingError as e:
            self.logerr(f"Failed to decode an incoming message: {e.message}")
            return
        # create CompressedImage message
        msg: ROSCompressedImage = ROSCompressedImage(
            header=rospy.Header(
                # TODO: reuse the timestamp from the incoming message
                stamp=rospy.Time.now(),
                frame_id=jpeg.header.frame,
            ),
            format=jpeg.format,
            data=jpeg.data,
        )
        # publish image
        self.pub_img.publish(msg)
        # ---
        if not self._has_published:
            self.log("Published the first image.")
            self._has_published = True

    async def publish_camera_info(self, rdata: RawData):
        try:
            camera: Camera = Camera.from_rawdata(rdata)
        except DataDecodingError as e:
            self.logerr(f"Failed to decode an incoming message: {e.message}")
            return
        msg: ROSCameraInfo = ROSCameraInfo(
            header=rospy.Header(
                # TODO: reuse the timestamp from the incoming message
                stamp=rospy.Time.now(),
                frame_id=camera.header.frame,
            ),
            width=camera.width,
            height=camera.height,
            distortion_model="plumb_bob",
            D=camera.D,
            K=camera.K,
            R=camera.R,
            P=camera.P,
        )
        self.pub_camera_info.publish(msg)

    async def worker(self):
        # create switchboard context
        switchboard = (await context("switchboard")).navigate(self._robot_name)
        # wait for camera to be ready
        jpeg = await (switchboard / "sensor" / "camera" / "jpeg").until_ready()
        parameters = await (switchboard / "sensor" / "camera" / "parameters").until_ready()
        # wait for camera parameters then publish
        rdata: RawData = await parameters.data_get()
        await self.publish_camera_info(rdata)
        # subscribe
        await jpeg.subscribe(self.publish)
        # ---
        await self.join()

    async def join(self):
        while not self.is_shutdown:
            await asyncio.sleep(1)

    def spin(self):
        try:
            asyncio.run(self.worker())
        except RuntimeError:
            if not self.is_shutdown:
                self.logerr("An error occurred while running the event loop")
                raise


if __name__ == "__main__":
    # initialize the node
    camera_node = CameraNode()
    # keep the node alive
    camera_node.spin()

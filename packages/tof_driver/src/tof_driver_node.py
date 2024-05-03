#!/usr/bin/env python3

import asyncio

import rospy
from sensor_msgs.msg import Range as ROSRange

from dt_robot_utils import get_robot_name
from dtps import context
from dtps_http import RawData
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_messages.sensors.range import Range
from duckietown_messages.utils.exceptions import DataDecodingError

MAX_RANGE = 99  # meters


class ToFNode(DTROS):
    def __init__(self):
        super(ToFNode, self).__init__(node_name="tof_node", node_type=NodeType.DRIVER)
        self._robot_name = get_robot_name()
        # arguments
        self._sensor_name: str = rospy.get_param("~sensor_name").replace("_", "-")
        # create publisher
        self._pub = rospy.Publisher(
            "~range",
            ROSRange,
            queue_size=1,
            dt_topic_type=TopicType.DRIVER,
            dt_help="The distance to the closest object detected by the sensor",
        )
        # user hardware test
        # self._hardware_test = HardwareTestToF(self._sensor_name, self._accuracy)

    async def publish(self, data: RawData):
        # TODO: only publish if somebody is listening
        # decode data
        try:
            tof: Range = Range.from_rawdata(data)
        except DataDecodingError as e:
            self.logerr(f"Failed to decode an incoming message: {e.message}")
            return
        # create Range message
        tof_msg: ROSRange = ROSRange(
            header=rospy.Header(
                # TODO: use the time from the incoming message
                stamp=rospy.Time.now(),
                frame_id=tof.header.frame,
            ),
            max_range=MAX_RANGE,
            range=tof.data if tof.data is not None else (MAX_RANGE + 1),
        )
        # publish messages
        self._pub.publish(tof_msg)

    async def worker(self):
        # create switchboard context
        switchboard = (await context("switchboard")).navigate(self._robot_name)
        # ToF queue
        tof = await (switchboard / "sensor" / "time-of-flight" / self._sensor_name / "range").until_ready()
        # subscribe
        await tof.subscribe(self.publish)
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

    def on_shutdown(self):
        loop: asyncio.AbstractEventLoop = asyncio.get_event_loop()
        if loop is not None:
            self.loginfo("Shutting down the event loop")
            loop.stop()


if __name__ == "__main__":
    # initialize the node
    node = ToFNode()
    # keep the node alive
    node.spin()

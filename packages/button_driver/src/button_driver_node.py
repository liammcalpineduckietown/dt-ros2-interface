#!/usr/bin/env python3

import asyncio

import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import ButtonEvent as ROSButtonEvent

from dt_robot_utils import get_robot_name
from dtps import context
from dtps_http import RawData
from duckietown_messages.sensors.button_event import ButtonEvent
from duckietown_messages.utils.exceptions import DataDecodingError


class ButtonDriverNode(DTROS):

    def __init__(self):
        super(ButtonDriverNode, self).__init__(
            node_name="button_driver_node",
            node_type=NodeType.DRIVER
        )
        self._robot_name = get_robot_name()

        # create publishers
        self.pub = rospy.Publisher(
            "~event",
            ROSButtonEvent,
            queue_size=1,
            dt_topic_type=TopicType.DRIVER,
            dt_help="Button event"
        )
        # ---
        self.loginfo("Initialized.")

    async def publish(self, data: RawData):
        # TODO: only publish if somebody is listening
        try:
            event: ButtonEvent = ButtonEvent.from_rawdata(data)
        except DataDecodingError as e:
            self.logerr(f"Failed to decode an incoming message: {e.message}")
            return
        # create ButtonEvent message
        msg: ROSButtonEvent = ROSButtonEvent(
            event=event.type.value
        )
        # publish
        self.pub.publish(msg)

    async def worker(self):
        # create switchboard context
        switchboard = (await context("switchboard")).navigate(self._robot_name)
        # wait for the button queue to be ready
        button = await (switchboard / "sensor" / "power-button").until_ready()
        # subscribe
        await button.subscribe(self.publish)
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
    node = ButtonDriverNode()
    # keep the node alive
    node.spin()

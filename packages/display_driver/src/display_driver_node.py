#!/usr/bin/env python3

import asyncio
from asyncio import AbstractEventLoop
from typing import Optional

import rospy
from duckietown_msgs.msg import DisplayFragment as DisplayFragmentMsg

from dt_robot_utils import get_robot_name
from dtps import context, DTPSContext
from dtps_http import RawData
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_messages.actuators.display_fragment import DisplayFragment
from duckietown_messages.actuators.display_fragments import DisplayFragments
from duckietown_messages.geometry_2d.roi import ROI
from duckietown_messages.sensors.image import Image


class DisplayDriverNode(DTROS):

    def __init__(self):
        super(DisplayDriverNode, self).__init__(node_name="display_driver_node", node_type=NodeType.DRIVER)
        self._robot_name = get_robot_name()
        # create subscribers
        self._fragments_sub = rospy.Subscriber(
            "~fragments",
            DisplayFragmentMsg,
            self.fragments_cb,
            queue_size=10,
            buf_size="4M",
            dt_topic_type=TopicType.DRIVER,
            dt_help="Data to display on the display",
        )
        # dtps publishers
        self._fragments: Optional[DTPSContext] = None
        # event loop
        self._loop: Optional[AbstractEventLoop] = None

    def fragments_cb(self, msg):
        """
        Callback that updates the display fragments.

            Args:
                msg (WheelsCmdStamped): wheel commands
        """
        if self._loop is None:
            return
        fragment = msg
        # pack data
        raw: RawData = DisplayFragments(
            fragments=[
                DisplayFragment(
                    name=fragment.id,
                    region=fragment.region,
                    page=fragment.page,
                    content=Image(
                        width=fragment.data.width,
                        height=fragment.data.height,
                        encoding=fragment.data.encoding,
                        is_bigendian=bool(fragment.data.is_bigendian),
                        step=fragment.data.step,
                        data=fragment.data.data,
                    ),
                    location=ROI(
                        x=fragment.location.x_offset,
                        y=fragment.location.y_offset,
                        width=fragment.location.width,
                        height=fragment.location.height,
                    ),
                    z=fragment.z,
                    ttl=fragment.ttl,
                )
            ]
        ).to_rawdata()
        # schedule the message for publishing
        # TODO: evaluate the efficiency of this approach, alternatively, use a coro that runs at 30Hz and keep
        #  publishing from a shared variable
        asyncio.run_coroutine_threadsafe(self._fragments.publish(raw), self._loop)

    async def worker(self):
        # create switchboard context
        switchboard = (await context("switchboard")).navigate(self._robot_name)
        # display fragments
        self._fragments = await (switchboard / "actuator" / "display" / "fragments").until_ready()
        # ---
        self._loop = asyncio.get_event_loop()
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
        if self._loop is not None:
            self.loginfo("Shutting down the event loop")
            self._loop.stop()


if __name__ == "__main__":
    # initialize the node
    node = DisplayDriverNode()
    # keep the node alive
    node.spin()

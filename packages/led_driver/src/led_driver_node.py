#!/usr/bin/env python3

import asyncio
from asyncio import AbstractEventLoop
from typing import Optional

import rospy
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA

from dt_robot_utils import get_robot_name
from dtps import DTPSContext, context
from dtps_http import RawData
from duckietown.dtros import DTROS, NodeType
from duckietown_messages.actuators.car_lights import CarLights
from duckietown_messages.colors.rgba import RGBA
from duckietown_messages.standard.header import Header


class LEDDriverNode(DTROS):
    """Node for controlling LEDs.

    Calls the low-level functions of class :obj:`RGB_LED` that creates the PWM
    signal used to change the color of the LEDs. The desired behavior is specified by
    the LED index (Duckiebots and watchtowers have multiple of these) and a pattern.
    A pattern is a combination of colors and blinking frequency.

    Duckiebots have 5 LEDs that are indexed and positioned as following:

        +------------------+------------------------------------------+
        | Index            | Position (rel. to direction of movement) |
        +==================+==========================================+
        | 0                | Front left                               |
        +------------------+------------------------------------------+
        | 1                | Rear left                                |
        +------------------+------------------------------------------+
        | 2                | Top / Front middle  (DB1X models only)   |
        +------------------+------------------------------------------+
        | 3                | Rear right                               |
        +------------------+------------------------------------------+
        | 4                | Front right                              |
        +------------------+------------------------------------------+

    """

    def __init__(self):
        super(LEDDriverNode, self).__init__(node_name="leds_driver", node_type=NodeType.DRIVER)
        self._robot_name = get_robot_name()
        # subscribers
        self.sub = rospy.Subscriber("~led_pattern", LEDPattern, self.led_cb, queue_size=1)
        # user hardware tests
        # self._hardware_test_front = HardwareTestLED(
        #     self.led, info_str="front", led_ids=[0, 2, 4], idle_lighting=self._idle
        # )
        # self._hardware_test_back = HardwareTestLED(
        #     self.led, info_str="back", led_ids=[1, 3], idle_lighting=self._idle
        # )
        # dtps publishers
        self._pattern: Optional[DTPSContext] = None
        # event loop
        self._loop: Optional[AbstractEventLoop] = None
        # ---
        self.log("Initialized.")

    def led_cb(self, msg):
        """
        Callback that enables/disables emergency stop

            Args:
                msg (BoolStamped): emergency_stop flag
        """
        if self._loop is None:
            return
        # make sure enough data is available
        if len(msg.rgb_vals) != 5:
            self.logerr(f"Invalid message. Expected 5 LED values, but got {len(msg.rgb_vals)}")
            return
        # pack data
        raw: RawData = CarLights(
            header=Header(
                # TODO: reuse the timestamp from the incoming message
            ),
            front_left=self._rgba(msg.rgb_vals[0]),
            front_right=self._rgba(msg.rgb_vals[4]),
            rear_left=self._rgba(msg.rgb_vals[1]),
            rear_right=self._rgba(msg.rgb_vals[3]),
        ).to_rawdata()
        # schedule the message for publishing
        asyncio.run_coroutine_threadsafe(self._pattern.publish(raw), self._loop)

    @staticmethod
    def _rgba(ros: ColorRGBA) -> RGBA:
        return RGBA(r=ros.r, g=ros.g, b=ros.b, a=ros.a)

    async def worker(self):
        # create switchboard context
        switchboard = (await context("switchboard")).navigate(self._robot_name)
        # leds pattern queue
        self._pattern = await (switchboard / "actuator" / "leds" / "rgba").until_ready()
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
    node = LEDDriverNode()
    # keep the node alive
    node.spin()

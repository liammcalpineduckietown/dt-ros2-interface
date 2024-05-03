#!/usr/bin/env python3

import asyncio
from asyncio import AbstractEventLoop
from typing import Optional

import rospy
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped

from dt_robot_utils import get_robot_name
from dtps import context, DTPSContext
from dtps_http import RawData
from duckietown.dtros import DTROS, TopicType, NodeType
from duckietown_messages.actuators.differential_pwm import DifferentialPWM
from duckietown_messages.standard.boolean import Boolean
from duckietown_messages.utils.exceptions import DataDecodingError


class WheelsDriverNode(DTROS):
    """
    Node that provides an interface to the robot's wheels.

    Subscribers:
       ~wheels_cmd (:obj:`WheelsCmdStamped`): The requested wheel command
       ~emergency_stop (:obj:`BoolStamped`): Emergency stop. Can stop the actual execution of
           the wheel commands by the motors if set to `True`. Set to `False` for nominal
           operations.
    Publishers:
       ~wheels_cmd_executed (:obj:`WheelsCmdStamped`): Publishes the actual commands executed,
           i.e. when the emergency flag is `False` it publishes the requested command, and
           when it is `True`: zero values for both motors.

    """

    def __init__(self):
        # Initialize the DTROS parent class
        super(WheelsDriverNode, self).__init__(node_name="wheels_driver", node_type=NodeType.DRIVER)
        self._robot_name = get_robot_name()
        # publishers
        self.pub_wheels_cmd = rospy.Publisher(
            "~wheels_cmd_executed", WheelsCmdStamped, queue_size=1, dt_topic_type=TopicType.DRIVER
        )
        # subscribers
        self.sub_topic = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.cmds_cb, queue_size=1)
        self.sub_e_stop = rospy.Subscriber("~emergency_stop", BoolStamped, self.estop_cb, queue_size=1)
        # user hardware tests
        # self._hardware_test_left = HardwareTestMotor(HardwareTestMotorSide.LEFT, self.driver)
        # self._hardware_test_right = HardwareTestMotor(HardwareTestMotorSide.RIGHT, self.driver)
        # dtps publishers
        self._pwm: Optional[DTPSContext] = None
        self._estop: Optional[DTPSContext] = None
        # event loop
        self._loop: Optional[AbstractEventLoop] = None
        # ---
        self.log("Initialized.")

    def estop_cb(self, msg):
        """
        Callback that enables/disables emergency stop

            Args:
                msg (BoolStamped): emergency_stop flag
        """
        if self._loop is None:
            return
        # pack data
        estop: bool = msg.data
        raw: RawData = Boolean(data=estop).to_rawdata()
        # schedule the message for publishing
        asyncio.run_coroutine_threadsafe(self._estop.publish(raw), self._loop)
        # log
        if estop:
            self.log("Emergency Stop Activated")
        else:
            self.log("Emergency Stop Released")

    def cmds_cb(self, msg):
        """
        Callback that updates the wheel commands.

            Args:
                msg (WheelsCmdStamped): wheel commands
        """
        if self._loop is None:
            return
        # pack data
        raw: RawData = DifferentialPWM(left=msg.vel_left, right=msg.vel_right).to_rawdata()
        # schedule the message for publishing
        # TODO: evaluate the efficiency of this approach, alternatively, use a coro that runs at 30Hz and keep
        #  publishing from a shared variable
        asyncio.run_coroutine_threadsafe(self._pwm.publish(raw), self._loop)

    async def publish_executed(self, data: RawData):
        # TODO: only publish if somebody is listening
        # decode data
        try:
            executed: DifferentialPWM = DifferentialPWM.from_rawdata(data)
        except DataDecodingError as e:
            self.logerr(f"Failed to decode an incoming message: {e.message}")
            return
        # create ROS message
        executed_msg: WheelsCmdStamped = WheelsCmdStamped(
            header=rospy.Header(
                # TODO: reuse the timestamp from the incoming message
                stamp=rospy.Time.now(),
                frame_id=executed.header.frame,
            ),
            vel_left=executed.left,
            vel_right=executed.right,
        )
        # publish messages
        self.pub_wheels_cmd.publish(executed_msg)

    async def worker(self):
        # create switchboard context
        switchboard = (await context("switchboard")).navigate(self._robot_name)
        # wheels PWM signal
        pwm_filtered = await (switchboard / "actuator" / "wheels" / "pwm_filtered").until_ready()
        # emergency stop
        self._estop = await (switchboard / "actuator" / "wheels" / "estop").until_ready()
        self._pwm = await (switchboard / "actuator" / "wheels" / "pwm").until_ready()
        # subscribe
        await pwm_filtered.subscribe(self.publish_executed)
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
    node = WheelsDriverNode()
    # keep the node alive
    node.spin()

#!/usr/bin/env python3

import asyncio
from typing import Optional
from duckietown_messages.actuators.drone_motor_command import DroneMotorCommand
from dtps.ergo_ui import DTPSContext

import rospy

from dt_robot_utils import get_robot_name
from dtps import context
from dtps_http import RawData
from duckietown.dtros import DTROS, NodeType
from duckietown_messages.utils.exceptions import DataDecodingError

from std_msgs.msg import Empty
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from duckietown_msgs.msg import DroneControl as DroneControlROS
from duckietown_msgs.msg import DroneMotorCommand as DroneMotorCommandROS
from duckietown_msgs.msg import DroneMode as DroneModeROS
from sensor_msgs.msg import BatteryState as BatteryStateROS
from duckietown_msgs.srv import SetDroneMode, SetDroneModeResponse
from duckietown_messages.sensors.battery import BatteryState
from duckietown_messages.actuators.drone_control import DroneControl
from duckietown_messages.actuators.drone_mode import DroneModeMsg, DroneModeResponse


class FlightControllerNode(DTROS):
    def __init__(self):
        super(FlightControllerNode, self).__init__(node_name="flight_controller_node", node_type=NodeType.DRIVER)
        self._robot_name = get_robot_name()

        # publishers initialization
        self._motor_pub = rospy.Publisher("~motors", DroneMotorCommandROS, queue_size=1)
        self._bat_pub = rospy.Publisher("~battery", BatteryStateROS, queue_size=1)
        self._mode_pub = rospy.Publisher("~mode/current", DroneModeROS, queue_size=1, latch=True)
        self._commands_pub = rospy.Publisher('~commands/executed', DroneControlROS, queue_size=1)
        
        # subscribers initialization
        rospy.Subscriber('~commands', DroneControlROS, self._fly_commands_cb, queue_size=1)
        rospy.Subscriber("~heartbeat/altitude", Empty, self._heartbeat_altitude_cb, queue_size=1)
        rospy.Subscriber("~heartbeat/joystick", Empty, self._heartbeat_joystick_cb, queue_size=1)
        rospy.Subscriber("~heartbeat/pid", Empty, self._heartbeat_pid_cb, queue_size=1)
        rospy.Subscriber("~heartbeat/state_estimator", Empty, self._heartbeat_state_estimator_cb,
                         queue_size=1)

        # ROS services
        self._srv_set_mode = rospy.Service("~set_mode", SetDroneMode, self._srv_set_mode_cb)
        self._srv_calib_imu = rospy.Service("~calibrate_imu", Trigger, self._srv_calibrate_imu_cb)
        self._srv_zero_yaw = rospy.Service("~zero_yaw", Trigger, self._srv_zero_yaw_cb)

        # dtps publishers
        self._commands: Optional[DTPSContext] = None
        self._hb_joystick_queue : Optional[DTPSContext] = None
        self._hb_pid_queue : Optional[DTPSContext] = None
        self._hb_altitude_queue : Optional[DTPSContext] = None
        self._hb_state_estimator_queue : Optional[DTPSContext] = None
        
        self._set_mode_queue: Optional[DTPSContext] = None
        self._calibrate_imu_queue: Optional[DTPSContext] = None
        self._zero_yaw_queue: Optional[DTPSContext] = None
        
        # event loop
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self.switchboard : Optional[DTPSContext] = None
        # ---
        self.loginfo("Initialized.")

    async def worker(self):
        # create switchboard context
        self.switchboard = (await context("switchboard")).navigate(self._robot_name)
        
        # Await queues to be ready
        self._commands = await (self.switchboard / "flight_controller" / "commands").until_ready()
        self._set_mode_queue = await (self.switchboard / "flight_controller" / "mode" / "set").until_ready()
        self._calibrate_imu_queue = await (self.switchboard / "imu" / "calibrate").until_ready()
        self._zero_yaw_queue = await (self.switchboard / "imu" / "zero_yaw").until_ready()
 
        self._hb_joystick_queue = await (self.switchboard / "heartbeat" / "joystick").until_ready()
        self._hb_pid_queue = await (self.switchboard / "heartbeat" / "pid").until_ready()
        self._hb_altitude_queue = await (self.switchboard / "heartbeat" / "altitude").until_ready()
        self._hb_state_estimator_queue = await (self.switchboard / "heartbeat" / "state_estimator").until_ready()
        
        battery_queue = await (self.switchboard / "sensor" / "battery").until_ready()
        motors_queue = await (self.switchboard / "actuator" / "motors").until_ready()
        executed_commands_queue = await (self.switchboard / "flight_controller" / "commands" / "executed").until_ready()
        mode_queue = await (self.switchboard / "flight_controller" / "mode" / "current").until_ready()

        # Subscribe
        await battery_queue.subscribe(self.publish_battery)
        await motors_queue.subscribe(self.publish_motor_pwm)
        await executed_commands_queue.subscribe(self.publish_commands)
        await mode_queue.subscribe(self.publish_mode)
        
        # ---
        self._loop = asyncio.get_event_loop()
        await self.join()

    async def publish_battery(self, data: RawData):
        # decode data
        try:
            battery = BatteryState.from_rawdata(data)
        except DataDecodingError as e:
            self.logerr(f"Failed to decode an incoming message: {e.message}")
            return
        # publish
        msg = BatteryStateROS()
        msg.header.stamp = rospy.Time.now()
        msg.voltage = battery.voltage
        msg.present = battery.present
        self._bat_pub.publish(msg)

    async def publish_motor_pwm(self, data: RawData):
        """
        Receives a message of type DroneControl from ROS and publishes it to the DTPS network
        """
        # decode data
        try:
            motor_msg : DroneMotorCommand = DroneMotorCommand.from_rawdata(data)
        except DataDecodingError as e:
            self.logerr(f"Failed to decode an incoming message: {e.message}")
            return
        # create ROS message
        motor_msg_ros = DroneMotorCommandROS()
        motor_msg_ros.header.stamp = rospy.Time.now()
        motor_msg.minimum = motor_msg.minimum
        motor_msg.maximum = motor_msg.maximum
        motor_msg_ros.m1 = motor_msg.m1
        motor_msg_ros.m2 = motor_msg.m2
        motor_msg_ros.m3 = motor_msg.m3
        motor_msg_ros.m4 = motor_msg.m4
        # publish messages
        self._motor_pub.publish(motor_msg_ros)
        
    async def publish_commands(self, data: RawData):
        # decode data
        try:
            commands = DroneControl.from_rawdata(data)
        except DataDecodingError as e:
            self.logerr(f"Failed to decode an incoming message: {e.message}")
            return
        # create ROS message
        msg = DroneControlROS()
        msg.header.stamp = rospy.Time.now()
        msg.roll = commands.roll
        msg.pitch = commands.pitch
        msg.yaw = commands.yaw
        msg.throttle = commands.throttle
        # publish
        self._commands_pub.publish(msg)

    async def publish_mode(self, data: RawData):
        # decode data
        try:
            mode_msg : DroneModeMsg = DroneModeMsg.from_rawdata(data)
        except DataDecodingError as e:
            self.logerr(f"Failed to decode an incoming message: {e.message}")
            return
        # create ROS message
        mode_msg_ros = DroneModeROS()
        mode_msg_ros.header.stamp = rospy.Time.now()
        mode_msg_ros.mode = mode_msg.mode
        # publish messages
        self._mode_pub.publish(mode_msg_ros)

    def _fly_commands_cb(self, msg: DroneControlROS):
        """
        Callback for the ~commands topic
        """
        if self._loop is None:
            return
        # create message
        commands = DroneControl(
            roll = msg.roll,
            pitch = msg.pitch,
            yaw = msg.yaw,
            throttle = msg.throttle
        )
        # send message
        self.publish_raw_data(self, commands.to_rawdata(), self._commands)

    async def join(self):
        while not self.is_shutdown:
            await asyncio.sleep(1)

    def _srv_set_mode_cb(self, req: SetDroneMode):
        """ Update desired mode """
        if self._loop is None:
            return

        mode = DroneModeMsg(mode=req.mode.mode)

        raw_data = asyncio.run_coroutine_threadsafe(
            self._set_mode_queue.call(
                    mode.to_rawdata()
                ),
            self._loop
            ).result()
        
        result : DroneModeResponse = DroneModeResponse.from_rawdata(
                raw_data)

        return SetDroneModeResponse(
            previous_mode=DroneModeROS(mode=result.previous_mode.value),
            current_mode=DroneModeROS(mode=result.current_mode.value),
        )

    def _srv_calibrate_imu_cb(self, _ : TriggerRequest):
        """ Calibrate IMU """
        try:
            raw_data = self.call_raw_data(RawData(b"", "text/plain"), self._calibrate_imu_queue)
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

        # respond
        response = raw_data.get_as_native_object()
        return TriggerResponse(success=response["success"], message=response["message"])
    
    def _srv_zero_yaw_cb(self, _):
        """ Zero yaw """
        try:
            raw_data = self.call_raw_data(RawData(b"", "text/plain"), self._zero_yaw_queue)
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

        # respond
        response = raw_data.get_as_native_object()
        return TriggerResponse(success=response["success"], message=response["message"])

    # heartbeat callbacks: These update the last time that data was received from a node

    def _heartbeat_joystick_cb(self, _):
        """ Update joystick heartbeat """
        self.publish_raw_data(self, RawData(b"", "text/plain"), self._hb_joystick_queue)

    def _heartbeat_pid_cb(self, _):
        """ Update pid_controller heartbeat """
        self.publish_raw_data(self, RawData(b"", "text/plain"), self._hb_pid_queue)

    def _heartbeat_altitude_cb(self, _):
        """ Update altitude sensor heartbeat """
        self.publish_raw_data(RawData(b"", "text/plain"), self._hb_altitude_queue)

    def _heartbeat_state_estimator_cb(self, _):
        """ Update state_estimator heartbeat """
        self.publish_raw_data(self, RawData(b"", "text/plain"), self._hb_state_estimator_queue)
        
    def spin(self):
        try:
            # Spin the sidecar workers
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
    
    def publish_raw_data(self, rd : RawData, queue : DTPSContext):
        """
        Utility method to publish raw data to a DTPS queue in a thread-safe manner.
        """
        if self._loop is None:
            return
        else:
            asyncio.run_coroutine_threadsafe(queue.publish(rd), self._loop)
        
    def call_raw_data(self, rd : RawData, service_queue : DTPSContext) -> RawData:
        """
        Utility method to call a DTPS queue in a thread-safe manner.
        """
        if self._loop is None:
            return
        else:
            return asyncio.run_coroutine_threadsafe(service_queue.call(rd), self._loop).result()

if __name__ == "__main__":
    # initialize the node
    node = FlightControllerNode()
    # keep the node alive
    node.spin()
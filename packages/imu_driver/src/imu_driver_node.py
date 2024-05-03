#!/usr/bin/env python3

import asyncio
from typing import Optional
from dt_robot_utils.constants import RobotType
from dt_robot_utils.robot import get_robot_type
from duckietown_messages.sensors.imu import Imu
from duckietown_messages.sensors.temperature import Temperature
from dtps.ergo_ui import DTPSContext
import rospy

from geometry_msgs.msg import Vector3, Quaternion
# from hardware_test_imu import HardwareTestIMU
from sensor_msgs.msg import (
    Imu as ROSImu,
    Temperature as ROSTemperature
)

from dt_robot_utils import get_robot_name
from dtps import context
from dtps_http import RawData
from duckietown.dtros import DTROS, NodeType
from duckietown_messages.utils.exceptions import DataDecodingError


class IMUNode(DTROS):
    def __init__(self):
        super(IMUNode, self).__init__(node_name="imu", node_type=NodeType.DRIVER)
        self._robot_name = get_robot_name()
        self._robot_type = get_robot_type()

        # publishers initialization
        self.pub_imu_raw = rospy.Publisher('~raw', ROSImu, queue_size=10)
        if self._robot_type == RobotType.DUCKIEDRONE:
            self.pub_imu_data = rospy.Publisher('~data', ROSImu, queue_size=10)
        
        if self._robot_type == RobotType.DUCKIEBOT:
            self.pub_therm = rospy.Publisher('~temperature', ROSTemperature, queue_size=10)

        # user hardware test
        # self._hardware_test = HardwareTestIMU()
        self._switchboard : Optional[DTPSContext] = None

        # ---
        self.loginfo("Initialized.")

    async def _publish_imu(self, data: RawData):
        # TODO: only publish if somebody is listening
        # decode data
        imu_data : Optional[Imu] = None

        try:
            imu_data = Imu.from_rawdata(data)
        except DataDecodingError as e:
            self.logerr(f"Failed to decode an incoming message: {e.message}")
            return

        # create IMU message
        imu_msg: ROSImu = ROSImu(
            header=rospy.Header(
                # TODO: reuse the timestamp from the incoming message
                stamp=rospy.Time.now(),
                frame_id=imu_data.header.frame,
            ),
            linear_acceleration=Vector3(
                x=imu_data.linear_acceleration.x,
                y=imu_data.linear_acceleration.y,
                z=imu_data.linear_acceleration.z,
            ),
            angular_velocity=Vector3(
                x=imu_data.angular_velocity.x,
                y=imu_data.angular_velocity.y,
                z=imu_data.angular_velocity.z,
                ),
        )

        self.pub_imu_raw.publish(imu_msg)

        if imu_data.orientation is not None:
            imu_msg.orientation = Quaternion(
                w=imu_data.orientation.w,
                x=imu_data.orientation.x,
                y=imu_data.orientation.y,
                z=imu_data.orientation.z,
            )

            # publish messages
            self.pub_imu_data.publish(imu_msg)

    async def _publish_temperature(self, data: RawData):
        # Decode data
        temperature_data : Temperature = Temperature.from_rawdata(data)
            
        # create temperature message
        therm_msg: ROSTemperature = ROSTemperature(
            header=rospy.Header(
                # TODO: reuse the timestamp from the incoming message
                stamp=rospy.Time.now(),
            ),
            temperature=temperature_data.data
        )
        self.pub_therm.publish(therm_msg)
        
    async def worker(self):
        # create switchboard context
        self._switchboard = (await context("switchboard")).navigate(self._robot_name)

        # IMU queue
        if self._robot_type == RobotType.DUCKIEDRONE:
            imu_queue = await (self._switchboard / "sensor" / "imu" / "data").until_ready()  
        else:
            imu_queue = await (self._switchboard / "sensor" / "imu" / "data_raw").until_ready()

        # subscribe
        await imu_queue.subscribe(self._publish_imu)
        # ---
        await self.join()
        
    async def worker_temperature(self):
        # The duckiebot has a temperature sensor
        temperature_queue = await (self._switchboard / "sensor" / "imu" / "temperature").until_ready()
        # Subscribe
        await temperature_queue.subscribe(self._publish_temperature)

        await self.join()
        

    async def join(self):
        while not self.is_shutdown:
            await asyncio.sleep(1)

    def spin(self):
        try:
            asyncio.run(self.main())
        except RuntimeError:
            if not self.is_shutdown:
                self.logerr("An error occurred while running the event loop")
                raise

    async def main(self):
        futures = [self.worker(),]
        if self._robot_type == RobotType.DUCKIEBOT:
            futures.append(self.worker_temperature())
        
        await asyncio.gather(*futures)

    def on_shutdown(self):
        loop: asyncio.AbstractEventLoop = asyncio.get_event_loop()
        if loop is not None:
            self.loginfo("Shutting down the event loop")
            loop.stop()


if __name__ == "__main__":
    # initialize the node
    node = IMUNode()
    # keep the node alive
    node.spin()

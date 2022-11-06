#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from Quanser.product_QCar import QCar

if __name__ == "__main__":
    rospy.init_node("IMU_publisher")
    pub = rospy.Publisher("qcar_imu/raw", Imu, queue_size=10)
    my_car = QCar()
    while not rospy.is_shutdown():
        imu = my_car.read_IMU()
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.linear_acceleration.x = imu[0][0]
        imu_msg.linear_acceleration.y = imu[0][1]
        imu_msg.linear_acceleration.z = imu[0][2]
        imu_msg.angular_velocity.x = imu[1][0]
        imu_msg.angular_velocity.y = imu[1][1]
        imu_msg.angular_velocity.z = imu[1][2]
        imu_msg.header.frame_id = "qcar_body"
        imu_msg.orientation_covariance[0] = -1 # set to -1 to indicate that orientation is not available
        pub.publish(imu_msg)
    rospy.spin()

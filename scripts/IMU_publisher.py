#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from Quanser.product_QCar import QCar

if __name__ == "__main__":
    rospy.init_node("IMU_publisher")
    pub = rospy.Publisher("qcar_imu/raw", Imu, queue_size=10)
    #pub = rospy.Publisher("/imu/data_raw", Imu, queue_size=10)
    my_car = QCar()
    while not rospy.is_shutdown():
        my_car.read_IMU()
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.linear_acceleration.x = my_car.read_other_buffer_IMU[3]
        imu_msg.linear_acceleration.y = my_car.read_other_buffer_IMU[4]
        imu_msg.linear_acceleration.z = my_car.read_other_buffer_IMU[5]
        imu_msg.angular_velocity.x = my_car.read_other_buffer_IMU[0]
        imu_msg.angular_velocity.y = my_car.read_other_buffer_IMU[1]
        imu_msg.angular_velocity.z = my_car.read_other_buffer_IMU[2]
        imu_msg.header.frame_id = "qcar_body"
        imu_msg.orientation_covariance[0] = -1 # set to -1 to indicate that orientation is not available
        pub.publish(imu_msg)
    rospy.spin()

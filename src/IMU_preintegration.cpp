//
// Created by qin on 10/29/22.
//
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include "../include/IMU_transport/IMU.h"
#include <glog/logging.h>

const float dt = 0.01; //TODO: get from IMU
// IMU初始化的开始条件
float min_time = 1;
//int min_measurements = 100;

bool
PreintegrateImuFromFirstKF(IMU::Preintegrated *current_preintegration, Eigen::Vector3f a, Eigen::Vector3f w, float dt) {
    current_preintegration->IntegrateNewMeasurement(a, w, dt);
    if (current_preintegration->GetIMUNum() <= min_imu_init) {
        current_preintegration->InitialiseDirectionG();
    }
    return true;
}

void imuReceiverCallback(const sensor_msgs::Imu::ConstPtr &msg, IMU::Preintegrated *preintegration) {
//    ROS_INFO("imu: %f, %f, %f, %f, %f, %f",
//             msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
//             msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    PreintegrateImuFromFirstKF(preintegration,
                               Eigen::Vector3f(msg->linear_acceleration.x, msg->linear_acceleration.y,
                                               msg->linear_acceleration.z),
                               Eigen::Vector3f(msg->angular_velocity.x, msg->angular_velocity.y,
                                               msg->angular_velocity.z), dt);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle node;
    tf2_ros::TransformBroadcaster pose_br;

//    Accelerometers calibration:
//    Scale Matrix
//    0.148654 00000000 00000000
//    00000000 0.289997 00000000
//    00000000 00000000 00.21814
//    Bias Vector
//    -134.829
//    030.3091
//    034.0347
//    Gyroscopes calibration: residual 0.823663
//    Scale Matrix
//    000.067685 0000000000 0000000000
//    0000000000 0.00491249 0000000000
//    0000000000 0000000000 000.304809
//    Bias Vector
//    -0.0112091
//    0.00421652
//    -0.0104901

    // IMU bias and calibration
    // for simulation data
    IMU::Bias b(0.007, 0.007, 0.007,
                0.006, 0.006, 0.006);
    Sophus::SE3<float> Tbc(Eigen::Matrix3f::Identity(), Eigen::Vector3f(0, 0, 0));
    IMU::Calib c(Tbc, 0.0001888339269965301, 0.000025019929573561175, 2.5565313322052523e-06, 6.972435158192731e-05);

    // for simulationGT data
//    IMU::Bias b(0, 0, 0, 0, 0, 0);
//    Sophus::SE3<float> Tbc(Eigen::Matrix3f::Identity(), Eigen::Vector3f(0, 0, 0));
//    IMU::Calib c(Tbc, 0.0, 0.0, 0.0, 0.0);

    auto *preintegrated = new IMU::Preintegrated(b, c);

    // 此处  _1 是占位符， 表示了const std_msgs::Imu::ConstPtr& msg
    ros::Subscriber subimu = node.subscribe<sensor_msgs::Imu>("qcar_imu/raw", 10,
                                                              boost::bind(&imuReceiverCallback, _1, preintegrated));
    geometry_msgs::TransformStamped imu_preintegrated_pose;
    ros::Publisher pub_quaternion = node.advertise<geometry_msgs::QuaternionStamped>("qcar_imu/pose/quaternion",
                                                                                     10);
    geometry_msgs::QuaternionStamped msg_quaternion;
    ros::Publisher pub_translation = node.advertise<geometry_msgs::PointStamped>("qcar_imu/pose/translation",
                                                                                 10);
    geometry_msgs::PointStamped msg_translation;

    while (ros::ok()) {
        if (preintegrated->GetIMUNum() > min_imu_init) { // If initialisation done
            Eigen::Matrix3f current_rotation = preintegrated->GetOriginalDeltaRotation();
            Eigen::Matrix3d current_rotation_d = current_rotation.cast<double>();
            Eigen::Quaterniond q(current_rotation_d);
            Eigen::Vector3f current_position = preintegrated->GetOriginalDeltaPosition();
            msg_quaternion.header.stamp = ros::Time::now(); // TODO: use the time of the latest imu message
            msg_quaternion.quaternion.x = q.x();
            msg_quaternion.quaternion.y = q.y();
            msg_quaternion.quaternion.z = q.z();
            msg_quaternion.quaternion.w = q.w();
            pub_quaternion.publish(msg_quaternion);
            msg_translation.header.stamp = msg_quaternion.header.stamp;
            msg_translation.point.x = current_position[0];
            msg_translation.point.y = current_position[1];
            msg_translation.point.z = current_position[2];
            pub_translation.publish(msg_translation);
            imu_preintegrated_pose.header.stamp = msg_quaternion.header.stamp;
            imu_preintegrated_pose.header.frame_id = "map";
            imu_preintegrated_pose.child_frame_id = "imu";
            imu_preintegrated_pose.transform.translation.x = current_position[0];
            imu_preintegrated_pose.transform.translation.y = current_position[1];
            imu_preintegrated_pose.transform.translation.z = current_position[2];
            imu_preintegrated_pose.transform.rotation.x = q.x();
            imu_preintegrated_pose.transform.rotation.y = q.y();
            imu_preintegrated_pose.transform.rotation.z = q.z();
            imu_preintegrated_pose.transform.rotation.w = q.w();
            pose_br.sendTransform(imu_preintegrated_pose);
        }
        ros::spinOnce();
    }

    return 0;
}
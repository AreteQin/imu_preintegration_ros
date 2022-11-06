//
// Created by qin on 10/29/22.
//
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
//#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
//#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include "../include/IMU_transport/IMU.h"
#include <glog/logging.h>

const float dt = 0.01; //TODO: get from IMU

bool PreintegrateImuFromLastKF() {return true;}

bool
PreintegrateImuFromFirstKF(IMU::Preintegrated *current_preintegration, Eigen::Vector3f a, Eigen::Vector3f w, float dt,
                           int &have_imu_num, Eigen::Matrix3f &Rwg, Eigen::Vector3f &dirG) {
    current_preintegration->IntegrateNewMeasurement(a, w, dt);
    // 初始化时关于速度的预积分定义Ri.t()*(s*Vj - s*Vi - Rwg*g*tij)
    dirG -= current_preintegration->GetOriginalDeltaRotation() * current_preintegration->GetOriginalDeltaVelocity();
    // 求取实际的速度，位移/时间
    Eigen::Vector3f _vel =
            current_preintegration->GetOriginalDeltaPosition() / current_preintegration->dT;
    if (have_imu_num < 6) {
        LOG(INFO) << "imu初始化失败, imu帧数量太少";
    } else {
        // dirG = sV1 - sVn + n*Rwg*g*t
        // 归一化，约等于重力在世界坐标系下的方向
        dirG = dirG / dirG.norm();
        // 原本的重力方向
        Eigen::Vector3f gI(0.0f, 0.0f, -1.0f);
        // 求重力在世界坐标系下的方向与重力在重力坐标系下的方向的叉乘
        Eigen::Vector3f v = gI.cross(dirG);
        // 求叉乘模长
        const float nv = v.norm();
        // 求转角大小
        const float cosg = gI.dot(dirG);
        const float ang = acos(cosg);
        // v/nv 表示垂直于两个向量的轴  ang 表示转的角度，组成角轴
        Eigen::Vector3f vzg = v * ang / nv;
        // 获得重力坐标系到世界坐标系的旋转矩阵的初值
        Rwg = Sophus::SO3f::exp(vzg).matrix();
        LOG(INFO) << "Rwg: " << std::endl << Rwg << std::endl;
    }
    have_imu_num = have_imu_num + 1;
    LOG(INFO)<< "have_imu_num: " << have_imu_num << std::endl;
    return true;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg, IMU::Preintegrated *preintegration, int &have_imu_num,
                 Eigen::Matrix3f &Rwg, Eigen::Vector3f &dirG) {
    ROS_INFO("imu: %f, %f, %f, %f, %f, %f",
             msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
             msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    PreintegrateImuFromFirstKF(preintegration,
                               Eigen::Vector3f(msg->linear_acceleration.x, msg->linear_acceleration.y,
                                               msg->linear_acceleration.z),
                               Eigen::Vector3f(msg->angular_velocity.x, msg->angular_velocity.y,
                                               msg->angular_velocity.z), dt, have_imu_num, Rwg, dirG);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle node;
    tf2_ros::TransformBroadcaster pose_br;

    // IMU初始化的开始条件
    float min_time = 1;
    int min_KF = 10;

    // IMU bias and calibration
    IMU::Bias b(0, 0, 0, 0, 0, 0);
    Sophus::SE3<float> Tbc(Eigen::Matrix3f::Identity(), Eigen::Vector3f(0, 0, 0));
    IMU::Calib c(Tbc, 0.04, 0.04, 0.01, 0.01);

    auto *preintegrated = new IMU::Preintegrated(b, c);

    // 重力系到世界系的旋转矩阵
    Eigen::Matrix3f Rwg;
    // 重力方向
    Eigen::Vector3f dirG;
    dirG.setZero();

    // Counter
    int have_imu_num = 0;

    ros::Subscriber subimu = node.subscribe<sensor_msgs::Imu>("qcar_imu/raw", 10,
                                                              boost::bind(&imuCallback, _1, preintegrated,
                                                                          have_imu_num, Rwg, dirG));
    // 此处  _1 是占位符， 表示了const std_msgs::Imu::ConstPtr& msg
    ros::Publisher pub_quaternion = node.advertise<geometry_msgs::QuaternionStamped>("qcar_imu/pose/quaternion",
                                                                                               10);
    geometry_msgs::QuaternionStamped msg_quaternion;
    ros::Publisher pub_translation = node.advertise<geometry_msgs::PointStamped>("qcar_imu/pose/translation",
                                                                                           10);
    geometry_msgs::PointStamped msg_translation;

    while (ros::ok()) {
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
        ros::spinOnce();
    }

    return 0;
}
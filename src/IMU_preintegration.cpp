//
// Created by qin on 10/29/22.
//
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "../include/IMU_transport/IMU.h"

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    ROS_INFO("imu: %f, %f, %f, %f, %f, %f",
             msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
             msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle node;
    ros::Subscriber subimu = node.subscribe("qcar_imu", 10, imuCallback);
    ros::spin();

    clock_t start_time, end_time;

    // Guassian noise
    float noise_mean_real = 0;
    float noise_standard_deviation_real = 0.05;
    std::default_random_engine e; //引擎
    std::normal_distribution<double> n(noise_mean_real, noise_standard_deviation_real); //均值, 标准差

    Eigen::Matrix<float, 6, 1> noise;
    Eigen::Vector3f a, a_real, w, w_real;
    std::vector<Eigen::Matrix<float, 6, 1>> all_measurements;

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
    float t = 0;
    end_time = clock();

    // Main loop
    while (t < 3) { // 3s
        // Generate noise
        noise << n(e), n(e), n(e), n(e), n(e), n(e);
//        std::cout << "noise: " << std::endl << noise << std::endl;
        // Generate imu measurement
        a_real << sin(t), cos(t), cos(t);
        w_real << sin(t), cos(t), sin(t);
        a << sin(t) + noise[0], cos(t) + noise[1], cos(t) + noise[2];
        w << sin(t) + noise[3], cos(t) + noise[4], sin(t) + noise[5];
//        Eigen::Matrix<float, 6, 1> meas;
//        meas << a, w;
//        std::cout << "imu_measurement: " << std::endl << a << std::endl << w << std::endl;
//        all_measurements.push_back(meas);
        // Get system time
        start_time = clock();
        preintegrated->IntegrateNewMeasurement(a, w, (float) (start_time - end_time) / CLOCKS_PER_SEC);
        std::cout << "dR： " << std::endl << preintegrated->GetOriginalDeltaRotation() << std::endl;
        t = t + (float) (start_time - end_time) / CLOCKS_PER_SEC;
        std::cout << "t: " << t << std::endl;

        have_imu_num++;
        // 初始化时关于速度的预积分定义Ri.t()*(s*Vj - s*Vi - Rwg*g*tij)
        dirG -= preintegrated->GetOriginalDeltaRotation() * preintegrated->GetOriginalDeltaVelocity();
        // 求取实际的速度，位移/时间
        Eigen::Vector3f _vel =
                preintegrated->GetOriginalDeltaPosition() / preintegrated->dT;
        if (have_imu_num < 6) {
            std::cout << "imu初始化失败, 由于带有imu预积分信息的关键帧数量太少" << std::endl;
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
            std::cout << "Rwg: " << std::endl << Rwg << std::endl;
//            mRwg = Rwg.cast<double>();
//            mTinit = mpCurrentKeyFrame->mTimeStamp-mFirstTs;
        }

        end_time = clock();
    }

    return 0;
}
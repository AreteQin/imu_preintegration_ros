//
// Created by qin on 10/29/22.
//
////////////////////////////////////////////////////////////
// 程序中变量名的第一个字母如果为"m"则表示为类中的成员变量，member
// 第一个、第二个字母:
// "p"表示指针数据类型
// "n"表示int类型
// "b"表示bool类型
// "s"表示set类型
// "v"表示vector数据类型
// 'l'表示list数据类型
// "KF"表示KeyFrame数据类型
////////////////////////////////////////////////////////////

#ifndef IMU_PREINTEGRATION_ROS_IMU_H
#define IMU_PREINTEGRATION_ROS_IMU_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <mutex>
#include <sophus/se3.hpp>
#include <glog/logging.h>

const int min_imu_init = 100;

namespace IMU {

    // 将不是标准正交的旋转矩阵单位正交化
    Eigen::Matrix3f NormalizeRotation(const Eigen::Matrix3f &R);

    Eigen::Matrix3f RightJacobianSO3(const float &x, const float &y, const float &z);

    Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f &v);

    Eigen::Matrix3f InverseRightJacobianSO3(const float &x, const float &y, const float &z);

    Eigen::Matrix3f InverseRightJacobianSO3(const Eigen::Vector3f &v);

    class point {
    };

    class Bias {
    public:
        Bias() : bax(0), bay(0), baz(0), bwx(0), bwy(0), bwz(0) {}

        Bias(const float &b_acc_x, const float &b_acc_y, const float &b_acc_z,
             const float &b_ang_vel_x, const float &b_ang_vel_y, const float &b_ang_vel_z) :
                bax(b_acc_x), bay(b_acc_y), baz(b_acc_z), bwx(b_ang_vel_x), bwy(b_ang_vel_y), bwz(b_ang_vel_z) {}

        void CopyFrom(Bias &b);

        friend std::ostream &operator<<(std::ostream &out, const Bias &b);

    public:
        float bax, bay, baz;
        float bwx, bwy, bwz;
    };

    class Calib {
    public:

        /**
        * @brief 设置参数
        * @param Tbc 位姿变换
        * @param ng 角速度计噪声
        * @param na 加速度计噪声
        * @param ngw 随机游走
        * @param naw 随机游走
        */
        Calib(const Sophus::SE3<float> &Tbc, const float &ng, const float &na, const float &ngw, const float &naw);

        /**
        * @brief imu标定参数的构造函数
        * @param calib imu标定参数
        */
        Calib(const Calib &calib);

        Calib() { mbIsSet = false; }

        //void Set(const cv::Mat &cvTbc, const float &ng, const float &na, const float &ngw, const float &naw);
        void
        Set(const Sophus::SE3<float> &sophTbc, const float &ng, const float &na, const float &ngw, const float &naw);

    public:
        // Sophus/Eigen implementation
        // Tcb: Transformation from camera to IMU
        Sophus::SE3<float> mTcb;
        Sophus::SE3<float> mTbc;
        // IMU intrinsic parameters: 噪声 和 随机游走 的协方差矩阵
        Eigen::DiagonalMatrix<float, 6> Cov, CovWalk;
        bool mbIsSet;
    };

    //Integration of 1 gyro measurement
    class IntegratedRotation {
    public:
        IntegratedRotation() {}

        // calculate the rotation matrix R_{j-1,j} from one gyro measurement and corresponding right Jacobian
        // equation (5.1) and (5.6.1)
        IntegratedRotation(const Eigen::Vector3f &angVel, const Bias &imuBias, const float &time);

    public:
        float deltaT{}; //integration time
        Eigen::Matrix3f deltaR;
        Eigen::Matrix3f rightJ; // right jacobian
    };

    //Preintegration of Imu Measurements
    class Preintegrated {
    public:
        Preintegrated(const Bias &b_, const Calib &calib);

        // copy constructor
        Preintegrated(Preintegrated *pImuPre) : dT(pImuPre->dT), C(pImuPre->C), Info(pImuPre->Info),
                                                Nga(pImuPre->Nga), NgaWalk(pImuPre->NgaWalk), b(pImuPre->b),
                                                dR(pImuPre->dR), dV(pImuPre->dV),
                                                dP(pImuPre->dP), JRg(pImuPre->JRg), JVg(pImuPre->JVg),
                                                JVa(pImuPre->JVa), JPg(pImuPre->JPg), JPa(pImuPre->JPa),
                                                avgA(pImuPre->avgA), avgW(pImuPre->avgW), bu(pImuPre->bu),
                                                db(pImuPre->db), mvMeasurements(pImuPre->mvMeasurements) {

        }

        Preintegrated() {}

        ~Preintegrated() = default;

        void CopyFrom(Preintegrated *pImuPre);

        void Initialize(const Bias &b_);

        // 当偏置更新时，对预计分进行更新，以便迭代优化
        void Reintegrate();

        // 累积新的IMU测量值到预计分结果，同时更新噪声
        void
        IntegrateNewMeasurement(const Eigen::Vector3f &acceleration, const Eigen::Vector3f &angVel, const float &dt);

        // Initialise the preintgration of IMU measurements
        bool InitialiseDirectionG();

        /**
        * @brief 融合两个预积分，发生在删除关键帧的时候，3帧变2帧，需要把两段预积分融合后重新计算预计分
        * @param pPrev 前面的预积分
        */
        void MergePrevious(Preintegrated *pPrev);

        // 更新bias
        void SetNewBias(const Bias &bu_);

        // Calculate the delta bias between the new one and the last one
        IMU::Bias GetDeltaBias(const Bias &b_);

        // Calculate the new delta rotation dR while updating the bias
        Eigen::Matrix3f GetDeltaRotation(const Bias &b_);

        // Calculate the new delta avgV dV while updating the bias
        Eigen::Vector3f GetDeltaVelocity(const Bias &b_);

        // Calculate the new delta position dP while updating the bias
        Eigen::Vector3f GetDeltaPosition(const Bias &b_);

//        // usage? TODO
//        Eigen::Matrix3f GetUpdatedDeltaRotation() {
//            std::unique_lock<std::mutex> lock(mMutex);
//            // equation (5.7)
//            return NormalizeRotation(dR * Sophus::SO3f::exp(JRg * db.head(3)).matrix());
//        }
//
//        Eigen::Vector3f GetUpdatedDeltaVelocity() {
//            std::unique_lock<std::mutex> lock(mMutex);
//            return dV + JVg * db.head(3) + JVa * db.tail(3);
//        }
//
//        Eigen::Vector3f GetUpdatedDeltaPosition() {
//            std::unique_lock<std::mutex> lock(mMutex);
//            return dP + JPg * db.head(3) + JPa * db.tail(3);
//        }
//
        Eigen::Matrix3f GetOriginalDeltaRotation();

        Eigen::Vector3f GetOriginalDeltaVelocity();

        Eigen::Vector3f GetOriginalDeltaPosition();

//
//        Bias GetOriginalBias() {
//            std::unique_lock<std::mutex> lock(mMutex);
//            return b;
//        }
//
//        Bias GetUpdatedBias() {
//            std::unique_lock<std::mutex> lock(mMutex);
//            return bu;
//        }
//
//        Eigen::Matrix<float, 6, 1> GetDeltaBias() {
//            std::unique_lock<std::mutex> lock(mMutex);
//            return db;
//        }
//
//        void printMeasurements() const {
//            std::cout << "pint meas:\n";
//            for (int i = 0; i < mvMeasurements.size(); i++) {
//                std::cout << "meas " << mvMeasurements[i].t << std::endl;
//            }
//            std::cout << "end pint meas:\n";
//        }

        int GetIMUNum();

        void UpdateDeltaPDeltaR();

    public:
        // the time between the first and last measurement in one preintegration
        float dT{};
        // 协方差矩阵
        Eigen::Matrix<float, 15, 15> C;
        // 信息矩阵
        Eigen::Matrix<float, 15, 15> Info;
        // Nga 6*6对角矩阵，3个陀螺仪噪声的平方，3个加速度计噪声的平方
        // 随机游走是已知的IMU内参，且不被更新
        Eigen::DiagonalMatrix<float, 6> Nga, NgaWalk;

        // Values for the original bias (when integration was computed)
        // bias before the update
        Bias b;
        Eigen::Matrix3f dR;
        Eigen::Vector3f dV, dP;
        // Jacobians wrt bias
        Eigen::Matrix3f JRg, JVg, JVa, JPg, JPa;
        Eigen::Vector3f avgA, avgW;
        // Average velocity
        Eigen::Vector3f avgV;
        // 重力系到世界系的旋转矩阵
        Eigen::Matrix3f Rwg;
        // 重力方向
        Eigen::Vector3f dirG; // unit vector
        float g = 9.81;
        Eigen::Vector3f accumulated_gravity;

    private:
        // Updated bias
        Bias bu;
        // Dif between original and updated bias
        // This is used to compute the updated values of the preintegration
        Eigen::Matrix<float, 6, 1> db;

        // Store one IMU measurement
        struct integrable {
            integrable() {}

            integrable(Eigen::Vector3f a_, Eigen::Vector3f w_, const float &t_) : a(std::move(a_)), w(std::move(w_)),
                                                                                  t(t_) {}

            Eigen::Vector3f a, w;
            float t;
        };

        std::vector<integrable> mvMeasurements;

        std::mutex mMutex;
    };
}


#endif //IMU_PREINTEGRATION_ROS_IMU_H

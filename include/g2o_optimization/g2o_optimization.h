#ifndef G2O_OPTIMIZATION_H_
#define G2O_OPTIMIZATION_H_

#include <vector>

#include "read_configs.h"
#include "camera.h"
#include "frame.h"
#include "mappoint.h"
#include "map.h"
#include "g2o_optimization/types.h"

/**
 * 图优化接口：调用者先用 types.h 中的数据描述状态与观测，再由优化函数创建 g2o 图。
 * 顶点表示待估计状态；边表示观测残差；目标为各边 rho(e^T * information * e) 的和。
 * fixed=true 的状态参与残差计算，但求解器不更新它；information 是残差的权重矩阵。
 * 坐标记号：T_ab 把 b 系坐标变换到 a 系；w=世界，c=左相机，b=IMU 机体。
 * poses 存相机到世界的 Rwc、twc；VertexVIPose 内部同时保存 Tcw 和 IMU 的 Twb。
 * 同一帧在 poses/velocities/biases 中共用帧 ID，建图时通过偏移生成不同的 g2o 顶点 ID。
 * 非 const 引用容器用于回写结果；优化函数内部同步求解，不负责启动线程或发布 ROS 消息。
 */
/// 提取 frame 的相机位姿到 poses，键为帧 ID；fix_this_frame 决定是否固定。
/// 此处只填充输入数据，不调用 optimizer.addVertex，也不执行优化。
void AddFrameVertex(FramePtr frame, MapOfPoses& poses, int id_camera, bool fix_this_frame);
/// 视觉惯性版本：同时提取位姿、世界系速度和 IMU 偏置。
/// add_imu_constraint=true 且存在前一帧时，记录“前一帧→当前帧”的预积分约束。
/// use_updated_bias=true 使用预积分线性化偏置加修正量，否则使用原线性化偏置。
void AddFrameVertex(FramePtr frame, MapOfPoses& poses, int id_camera, MapOfVelocity& velocities, MapOfBias& biases, 
    VectorOfIMUConstraints& imu_constraints, bool fix_this_frame, bool add_imu_constraint, bool use_updated_bias=false);

// pose, velocaity and bias of same frame have same key, i.e. id.
/// 局部 BA：联合优化未固定的多帧位姿、地图点/线，以及传入的速度、偏置。
/// 使用单目/双目重投影边；有 IMU 约束时再加入预积分与偏置差分边，重力方向固定。
/// 按 cfg 配置鲁棒核/外点阈值，回写状态及观测 inlier；固定状态用于锚定局部地图。
void LocalmapOptimization(MapOfPoses& poses, MapOfPoints3d& points, MapOfLine3d& lines, 
    MapOfVelocity& velocities, MapOfBias& biases, std::vector<CameraPtr>& camera_list, 
    VectorOfMonoPointConstraints& mono_point_constraints, VectorOfStereoPointConstraints& stereo_point_constraints, 
    VectorOfMonoLineConstraints& mono_line_constraints, VectorOfStereoLineConstraints& stereo_line_constraints,
    VectorOfIMUConstraints& imu_constraints, const Eigen::Matrix3d& Rwg, const OptimizationConfig& cfg);

// 单帧跟踪优化
/// 跟踪阶段的帧状态优化：创建 g2o 图，用 LM 求解，并返回最终视觉内点数量。
/// MapBuilder::FramePoseOptimization 调用本函数：地图点固定，当前帧位姿可更新；
/// IMU 已初始化且预积分有效时还传入参考帧、速度、偏置和 IMU 约束。
/// 接口也支持线约束，但当前 MapBuilder 调用传入的线约束为空。
/// 当前实现最多进行 3 轮、每轮最多 10 次迭代，轮间按 chi2 标记外点；结果写回容器。
int FrameOptimization(MapOfPoses& poses, MapOfPoints3d& points, MapOfLine3d& lines,
    MapOfVelocity& velocities, MapOfBias& biases, std::vector<CameraPtr>& camera_list, 
    VectorOfMonoPointConstraints& mono_point_constraints, VectorOfStereoPointConstraints& stereo_point_constraints, 
    VectorOfMonoLineConstraints& mono_line_constraints, VectorOfStereoLineConstraints& stereo_line_constraints,
    VectorOfIMUConstraints& imu_constraints, Eigen::Matrix3d& Rwg, const OptimizationConfig& cfg);

// IMU 初始化
/// 视觉惯性对齐：在输入视觉位姿上构建 IMU 图，估计各帧速度、共享偏置和重力方向。
/// 位姿是否固定由 poses.fixed 决定；此函数不加入视觉重投影边，也不估计单目尺度。
/// 对偏置加入先验，LM 最多迭代 200 次；回写 velocities、bias、Rwg。
/// 当前实现末尾直接返回 true；返回值本身不是收敛或运动激励充分性的判据。
bool IMUInitialization(MapOfPoses& poses, MapOfVelocity& velocities, Bias& bias, std::vector<CameraPtr>& camera_list,
    VectorOfIMUConstraints& imu_constraints, Eigen::Matrix3d& Rwg);


// 视觉 PnP 位姿估计
/// 由世界系地图点和当前帧像素调用 OpenCV solvePnPRansac，提供视觉初始位姿。
/// 输出 pose 为 Twc；inliers 与输入 mappoints 对齐，内点填地图点 ID，其余为 -1。
/// 返回 RANSAC 内点数，失败返回 0；此函数不运行 g2o。
int SolvePnPWithCV(FramePtr frame, std::vector<MappointPtr>& mappoints, Eigen::Matrix4d& pose, std::vector<int>& inliers);

// frames: [n, n-1, n-2....]
/// 比较视觉相对旋转和 IMU 预积分旋转，利用 JRg 解线性最小二乘偏置修正 dbg。
/// frames 按 [最新帧, 次新帧, ...] 排列，需有相邻帧及有效预积分。
bool ComputeGyrBias(std::vector<FramePtr>& frames, Eigen::Vector3d& dbg);
/// 根据视觉位移、预积分 dV/dP 联立线性方程，估计各帧世界系速度和重力向量 gw。
/// frames 按新到旧排列；速度直接写回 Frame。这里只做线性初估，不调用 g2o。
bool ComputeVelocity(std::vector<FramePtr>& frames, Eigen::Vector3d& gw);

// frames: [n, n-1, n-2....], for debugging
/// 调试输出：打印相邻帧的视觉旋转与预积分旋转之差，用于检查陀螺仪偏置。
void ValidateGyrBias(std::vector<FramePtr>& frames); 
/// 调试输出：检查速度/重力线性方程；x 布局为各帧 3 维速度，末尾 3 维为重力。
void ValidateVelocity(std::vector<FramePtr>& frames, Eigen::VectorXd x);
/// 调试输出：用帧状态和给定世界系重力 gw，计算各段 IMU 旋转、速度、位置残差。
void ValidateError(std::vector<FramePtr>& frames, const Eigen::Vector3d& gw);
/// 调试输出：从优化容器重算 IMU 残差以检查坐标系、偏置及对齐结果。
/// 当前实现仅打印并返回 true，不按误差阈值作初始化成功判定。
bool ValidateError(MapOfPoses& poses, MapOfVelocity& velocities, Bias& bias, std::vector<CameraPtr>& camera_list,
    VectorOfIMUConstraints& imu_constraints, Eigen::Matrix3d& Rwg, const Eigen::Vector3d& prior_gyr_bias, const Eigen::Vector3d& prior_acc_bias);
/// 调试输出：用前一帧状态和预积分预测后一帧，比较旋转、速度、位置的一致性。
void ValidateIMUInitialization(std::vector<FramePtr>& frames);


// for offline map refinement
/// 位姿图优化：只建立帧位姿顶点和相对位姿边，最多迭代 20 次；回写 poses。
/// 不优化地图点/线；全局参考系由调用者通过 poses.fixed 锚定。
void PoseGraphOptimization(MapOfPoses& poses, std::vector<CameraPtr>& camera_list, VectorOfRelativePoseConstraints& relative_pose_constraints);
/// 离线全局视觉 BA：直接读取并更新 Map 中的关键帧、地图点和地图线。
/// point/line_outlier_rejection 控制对应观测的外点剔除；first/second_iterations
/// 控制两阶段求解次数。此接口没有速度、偏置或 IMU 预积分约束输入。
void GlobalBA(MapPtr _map, const OptimizationConfig& cfg, bool point_outlier_rejection, 
    bool line_outlier_rejection, int first_iterations, int second_iterations);

#endif  // G2O_OPTIMIZATION_H_
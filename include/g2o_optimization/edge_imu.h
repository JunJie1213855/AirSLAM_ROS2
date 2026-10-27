#ifndef EDGE_IMU_H_
#define EDGE_IMU_H_

#include <Eigen/Core> 
#include <Eigen/Geometry> 
#include <opencv2/core/eigen.hpp>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/sba/g2o_types_sba_api.h>
#include <g2o/types/sba/vertex_se3_expmap.h>
#include <g2o/types/slam3d_addons/line3d.h>
#include <g2o/core/base_multi_edge.h>

#include "utils.h"
#include "imu.h"
#include "g2o_optimization/vertex_imu.h"
#include "g2o_optimization/vertex_vi_pose.h"

/**
 * 9 维 IMU 预积分边，顶点索引严格为：
 * [0: pose_i, 1: velocity_i, 2: gyro_bias, 3: acc_bias,
 *  4: pose_j, 5: velocity_j, 6: gravity_direction]。
 * 偏置用于修正 i→j 的预积分量；通常连接 i 时刻偏置，初始化时可共享。
 * 残差顺序 [旋转(3), 速度(3), 位置(3)]：
 * er=Log(dR^T * Rwi^T * Rwj)
 * ev=Rwi^T*(vj-vi-gw*dt)-dV
 * ep=Rwi^T*(pj-pi-vi*dt-0.5*gw*dt^2)-dP。
 * dR/dV/dP 已按偏置修正；信息矩阵来自预积分 9x9 协方差的逆。
 * 偏置随机游走单独用 EdgeGyr/EdgeAcc 描述，不包含在这 9 维残差中。
 */
class EdgeIMU : public g2o::BaseMultiEdge<9, Vector9d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeIMU(PreinterationPtr preinteration_); 

  /// 本类未实现文件反序列化，调用返回 false。
  virtual bool read(std::istream& is){return false;}
  /// 本类未实现文件序列化，调用返回 false。
  virtual bool write(std::ostream& os) const{return false;}
  /// g2o 在迭代/误差检查时调用，按当前顶点估计更新 _error；不直接更新顶点。
  void computeError();
  /// 按当前顶点状态重算并打印三类残差，供调试，不执行优化。
  void PrintError();

  Eigen::Vector3d g;
  PreinterationPtr preinteration;
};

/// 陀螺仪偏置差分边：[0: bg_i, 1: bg_j]，残差 bg_j-bg_i（3 维）。
/// 用于偏置随机游走；把一个顶点固定时也可用作偏置先验；不减 _measurement。
class EdgeGyr : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexGyrBias, VertexGyrBias>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeGyr(){}

  /// 本类未实现文件反序列化，调用返回 false。
  virtual bool read(std::istream& is){return false;}
  /// 本类未实现文件序列化，调用返回 false。
  virtual bool write(std::ostream& os) const{return false;}

  /// g2o 在迭代/误差检查时调用，按当前顶点估计更新 _error；不直接更新顶点。
  void computeError();
};

/// 加速度计偏置差分边：[0: ba_i, 1: ba_j]，残差 ba_j-ba_i（3 维）。
/// 用于偏置随机游走或固定顶点先验；权重由建图处设置，不减 _measurement。
class EdgeAcc : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexAccBias, VertexAccBias>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeAcc(){}

  /// 本类未实现文件反序列化，调用返回 false。
  virtual bool read(std::istream& is){return false;}
  /// 本类未实现文件序列化，调用返回 false。
  virtual bool write(std::ostream& os) const{return false;}

  /// g2o 在迭代/误差检查时调用，按当前顶点估计更新 _error；不直接更新顶点。
  void computeError();
};

#endif  // EDGE_IMU_H_

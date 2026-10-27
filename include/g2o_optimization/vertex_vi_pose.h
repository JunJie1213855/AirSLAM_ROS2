#ifndef VERTEX_VI_POSE_H_
#define VERTEX_VI_POSE_H_

#include <Eigen/Core> 
#include <Eigen/Geometry> 
#include <opencv2/core/eigen.hpp>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/sba/g2o_types_sba_api.h>
#include <g2o/types/sba/vertex_se3_expmap.h>
#include <g2o/types/slam3d_addons/line3d.h>

/**
 * 同一刚体位姿的相机/IMU 两种表达。
 * Rcw,tcw,Tcw：世界→相机；Rcb,tcb：IMU→相机固定外参；Rbc,tbc 为其逆。
 * Rwb,twb：IMU→世界，twb 是 IMU 在世界中的位置；tcw 不是相机世界位置。
 */
class VIPose{
public:
  VIPose();
  VIPose(const Eigen::Matrix3d& Rcw_, const Eigen::Vector3d& tcw_, const Eigen::Matrix3d& Rcb_, const Eigen::Vector3d& tcb_);
  VIPose& operator =(const VIPose& other);
  /// 由 Tcw 和固定 Tcb 外参设置状态，并推导 Twb、Tbc 等缓存。
  void SetParam(const Eigen::Matrix3d& Rcw_, const Eigen::Vector3d& tcw_, const Eigen::Matrix3d& Rcb_, const Eigen::Vector3d& tcb_);
  /// 右乘旋转 Rwb <- Rwb*Exp(dr)，位置 twb <- twb+Rwb*dt，再同步 Tcw。
  void Update(const double* v);

public:
  Eigen::Matrix3d Rcw, Rcb, Rbc, Rwb;
  Eigen::Vector3d tcw, tcb, tbc, twb;
  g2o::SE3Quat Tcw;
  // 累计局部更新次数，用于周期性归一化旋转矩阵。
  int n_it;
};

/// 6 自由度位姿顶点，保存 VIPose；视觉边读 Tcw，IMU 边读 Rwb/twb。
/// 局部增量顺序为 [旋转 dr(3), 平移 dt(3)]，通过 VIPose::Update 更新。
class VertexVIPose : public g2o::BaseVertex<6, VIPose> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexVIPose();
  /// 本类未实现文件反序列化，调用返回 false。
  virtual bool read(std::istream& is);
  /// 本类未实现文件序列化，调用返回 false。
  virtual bool write(std::ostream& os) const;

  /// g2o 原点重置接口；本类当前实现为空，不改变状态。
  virtual void setToOriginImpl() {}

  /// g2o 将求出的局部增量交给此接口，按该状态的更新规则修改 _estimate。
  virtual void oplusImpl(const double* update_) {
    _estimate.Update(update_);
    updateCache();
  }

  /// 返回此顶点的估计维数。
  virtual int estimateDimension() const { return 6; }
};

#endif  // VERTEX_VI_POSE_H_
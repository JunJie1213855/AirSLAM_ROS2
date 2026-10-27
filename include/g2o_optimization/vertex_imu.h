#ifndef VERTEX_IMU_H_
#define VERTEX_IMU_H_

#include <Eigen/Core> 
#include <Eigen/Geometry> 
#include <opencv2/core/eigen.hpp>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/sba/g2o_types_sba_api.h>
#include <g2o/types/sba/vertex_se3_expmap.h>
#include <g2o/types/slam3d_addons/line3d.h>


/// 3 维 IMU 世界系速度 v_w；采用普通向量加法更新，单位 m/s。
class VertexVelocity : public g2o::BaseVertex<3, Eigen::Vector3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexVelocity();
  VertexVelocity(const Eigen::Vector3d& velocity);
  /// 从流读取顶点估计，不参与求解迭代。
  virtual bool read(std::istream& is);
  /// 向流写出顶点估计。
  virtual bool write(std::ostream& os) const;

  /// g2o 原点重置接口：将该状态恢复为零向量。
  virtual void setToOriginImpl() { _estimate.setZero(); }
  /// g2o 将求出的局部增量交给此接口，按该状态的更新规则修改 _estimate。
  virtual void oplusImpl(const double* update_);
  /// 返回此顶点的估计维数。
  virtual int estimateDimension() const;
};


/// 3 维陀螺仪偏置 b_g，表达在 IMU 测量轴上，单位 rad/s；加法更新。
class VertexGyrBias : public g2o::BaseVertex<3, Eigen::Vector3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexGyrBias();
  VertexGyrBias(const Eigen::Vector3d& gyr_bias);
  /// 从流读取顶点估计，不参与求解迭代。
  virtual bool read(std::istream& is);
  /// 向流写出顶点估计。
  virtual bool write(std::ostream& os) const;

  /// g2o 原点重置接口：将该状态恢复为零向量。
  virtual void setToOriginImpl();
  /// g2o 将求出的局部增量交给此接口，按该状态的更新规则修改 _estimate。
  virtual void oplusImpl(const double* update_);
  /// 返回此顶点的估计维数。
  virtual int estimateDimension() const;
};


/// 3 维加速度计偏置 b_a，表达在 IMU 测量轴上，单位 m/s^2；加法更新。
class VertexAccBias : public g2o::BaseVertex<3, Eigen::Vector3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexAccBias();
  VertexAccBias(const Eigen::Vector3d& acc_bias);
  /// 从流读取顶点估计，不参与求解迭代。
  virtual bool read(std::istream& is);
  /// 向流写出顶点估计。
  virtual bool write(std::ostream& os) const;

  /// g2o 原点重置接口：将该状态恢复为零向量。
  virtual void setToOriginImpl();
  /// g2o 将求出的局部增量交给此接口，按该状态的更新规则修改 _estimate。
  virtual void oplusImpl(const double* update_);
  /// 返回此顶点的估计维数。
  virtual int estimateDimension() const;
};

/// 重力方向状态：g_w=Rwg*[0,0,-IMU_G_VALUE]^T，重力模长保持不变。
/// Update 右乘 Exp([dx,dy,0])，仅用 2 维倾斜增量；重力本身不能约束航向。
class GDirection{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GDirection();
  GDirection(const Eigen::Matrix3d& Rwg_);

  /// 用 2 维倾斜增量右乘更新 Rwg，保持重力模长不变。
  void Update(const double *update_);

  Eigen::Matrix3d Rwg;
  int its;
};

/// 2 自由度重力方向顶点，初始化时可优化，常规跟踪/局部 BA 中固定。
class VertexGDirection : public g2o::BaseVertex<2, GDirection>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexGDirection();
  VertexGDirection(const Eigen::Matrix3d& Rwg);

  /// 本类未实现文件反序列化，调用返回 false。
  virtual bool read(std::istream &is);
  /// 本类未实现文件序列化，调用返回 false。
  virtual bool write(std::ostream &os) const;

  /// g2o 原点重置接口；本类当前实现为空，不改变状态。
  virtual void setToOriginImpl();
  /// g2o 将求出的局部增量交给此接口，按该状态的更新规则修改 _estimate。
  virtual void oplusImpl(const double *update_);
};


#endif  // VERTEX_IMU_H_
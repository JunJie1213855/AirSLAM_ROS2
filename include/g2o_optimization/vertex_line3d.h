#ifndef VERTEX_LINE_3D_H_
#define VERTEX_LINE_3D_H_

#include <Eigen/Core> 
#include <Eigen/Geometry> 
#include <opencv2/core/eigen.hpp>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/sba/g2o_types_sba_api.h>
#include <g2o/types/sba/vertex_se3_expmap.h>
#include <g2o/types/slam3d_addons/line3d.h>

/// 世界系三维无限直线顶点：g2o::Line3D 以 6 维 Plucker 坐标保存，
/// 但直线只有 4 个独立自由度，因此 BaseVertex 的优化增量维度为 4。
/// 图像线段的端点只是观测，不是此顶点直接优化的两个三维端点。
class VertexLine3D : public g2o::BaseVertex<4, g2o::Line3D> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexLine3D();
  /// 从流读取顶点估计，不参与求解迭代。
  virtual bool read(std::istream& is);
  /// 向流写出顶点估计。
  virtual bool write(std::ostream& os) const;

  /// g2o 原点重置接口：将该状态恢复为零向量或默认直线。
  virtual void setToOriginImpl() { _estimate = g2o::Line3D(); }

  /// g2o 将求出的局部增量交给此接口，按该状态的更新规则修改 _estimate。
  virtual void oplusImpl(const double* update_) {
    Eigen::Map<const Eigen::Vector4d> update(update_);
    _estimate.oplus(update);
  }

  /// 从 6 维直线存储数据设置估计，与 4 维局部优化增量区分。
  virtual bool setEstimateDataImpl(const double* est) {
    Eigen::Map<const Vector6d> _est(est);
    _estimate = g2o::Line3D(_est);
    return true;
  }

  /// 导出完整的 6 维 Plucker 直线坐标。
  virtual bool getEstimateData(double* est) const {
    Eigen::Map<Vector6d> _est(est);
    _est = _estimate;
    return true;
  }

  /// 完整估计的存储维数；直线的存储维数可与局部增量维数不同。
  virtual int estimateDimension() const { return 6; }

  // 可视化颜色，不是优化变量。
  Eigen::Vector3d color;
};

#endif  // VERTEX_LINE_3D_H_
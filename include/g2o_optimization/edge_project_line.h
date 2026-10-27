#ifndef EDGE_PROJECT_LINE_H_
#define EDGE_PROJECT_LINE_H_

#include <Eigen/Core> 
#include <Eigen/Geometry> 
#include <opencv2/core/eigen.hpp>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/sba/g2o_types_sba_api.h>
#include <g2o/types/sba/vertex_se3_expmap.h>
#include <g2o/types/slam3d_addons/line3d.h>

#include "utils.h"
#include "g2o_optimization/vertex_vi_pose.h"
#include "g2o_optimization/vertex_line3d.h"

// 
/// 单目线重投影边，连接 [0: 世界系 VertexLine3D, 1: VertexVIPose]。
/// 测量是线段两端点 [u1,v1,u2,v2]（4 维），残差是两端点到预测直线
/// l=[a,b,c] 的有符号距离 (a*u+b*v+c)/sqrt(a*a+b*b)（2 维，像素）。
/// 注意 BaseBinaryEdge 的首个维数是残差维数，不要求等于测量的维数。
class EdgeSE3ProjectLine
    : public g2o::BaseBinaryEdge<2, Eigen::Vector4d, VertexLine3D, VertexVIPose> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectLine();

  /// 从流读取观测与信息矩阵，不参与求解迭代。
  bool read(std::istream &is);
  /// 向流写出观测与信息矩阵。
  bool write(std::ostream &os) const;
  /// g2o 在迭代/误差检查时调用，按当前顶点估计更新 _error；不直接更新顶点。
  void computeError();

  /// 将相机系三维直线投影为齐次图像直线 [a,b,c]。
  Eigen::Vector3d cam_project(const g2o::Line3D &line) const;

  double fx, fy;
  Eigen::Vector3d Kv; // [-cx*fy, -fx*cy, fx*fy]
};


/// 双目线重投影边，顶点顺序同单目线边。
/// 测量为左目两端点与右目两端点拼接（8 维），残差为四个点到各自
/// 预测直线的有符号距离（4 维）；右目由左目外参沿 x 平移 -b 得到。
class EdgeStereoSE3ProjectLine
    : public g2o::BaseBinaryEdge<4, Vector8d, VertexLine3D, VertexVIPose> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeStereoSE3ProjectLine();

  /// 从流读取观测与信息矩阵，不参与求解迭代。
  bool read(std::istream &is);
  /// 向流写出观测与信息矩阵。
  bool write(std::ostream &os) const;
  /// g2o 在迭代/误差检查时调用，按当前顶点估计更新 _error；不直接更新顶点。
  void computeError();

  /// 将相机系三维直线投影为齐次图像直线 [a,b,c]。
  Eigen::Vector3d cam_project(const g2o::Line3D &line) const;

  // b 为物理基线（米），不同于点边中的 bf=fx*b。
  double fx, fy, b;
  Eigen::Vector3d Kv; // [-cx*fy, -fx*cy, fx*fy]
};

#endif  // EDGE_PROJECT_LINE_H_
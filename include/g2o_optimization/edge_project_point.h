#ifndef EDGE_PROJECT_POINT_H_
#define EDGE_PROJECT_POINT_H_

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
// 
/// 单目点重投影边，连接 [0: 世界系 VertexPointXYZ, 1: VertexVIPose]。
/// 测量 [u,v]，残差 e=观测-pi(Rcw*Pw+tcw)，共 2 维，单位像素。
/// 使用针孔投影，输入观测应与相机内参/畸变校正约定一致。
class EdgeSE3ProjectPoint
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ, VertexVIPose> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectPoint();

  /// 从流读取观测与信息矩阵，不参与求解迭代。
  bool read(std::istream &is);
  /// 向流写出观测与信息矩阵。
  bool write(std::ostream &os) const;
  /// g2o 在迭代/误差检查时调用，按当前顶点估计更新 _error；不直接更新顶点。
  void computeError();
  /// 检查地图点变换到左相机后 Z>0，用于排除相机后方的无效投影。
  bool isDepthPositive();

  /// 将相机系三维点投影为 像素观测；不负责世界到相机的坐标变换。
  Eigen::Vector2d cam_project(const Eigen::Vector3d& point) const;
  // Eigen::Matrix<double, 2, 3> cam_projection_jacobian(const Eigen::Vector3d &point) const;
  // virtual void linearizeOplus();

  double fx, fy, cx, cy;
};

// 
/// 校正双目点重投影边，顶点顺序同单目点边。
/// 测量 [u_left,v_left,u_right]，残差为观测减预测，共 3 维像素误差。
/// 右目预测 u_right=u_left-bf/Z；bf=fx*基线，假定双目水平校正。
class EdgeSE3ProjectStereoPoint
    : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexPointXYZ, VertexVIPose> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectStereoPoint();

  /// 从流读取观测与信息矩阵，不参与求解迭代。
  bool read(std::istream &is);
  /// 向流写出观测与信息矩阵。
  bool write(std::ostream &os) const;
  /// g2o 在迭代/误差检查时调用，按当前顶点估计更新 _error；不直接更新顶点。
  void computeError();
  /// 检查地图点变换到左相机后 Z>0，用于排除相机后方的无效投影。
  bool isDepthPositive();

  /// 将相机系三维点投影为 像素观测；不负责世界到相机的坐标变换。
  Eigen::Vector3d cam_project(const Eigen::Vector3d& point) const;
  // Eigen::Matrix<double, 2, 3> cam_projection_jacobian(const Eigen::Vector3d &point) const;
  // virtual void linearizeOplus();
  
  double fx, fy, cx, cy, bf;
};

#endif  // EDGE_PROJECT_POINT_H_

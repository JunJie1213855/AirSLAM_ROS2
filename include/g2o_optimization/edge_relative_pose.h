#ifndef EDGE_RELATIVE_POSE_H_
#define EDGE_RELATIVE_POSE_H_

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

/// 6 维相对位姿边，连接 [0: pose_1, 1: pose_2]，用于位姿图优化。
/// Rc1c2,tc1c2 保存测量 T_c1_c2（相机 2→相机 1），不通过 _measurement 读取。
/// 预测旋转为 R_c1w*R_c2w^T，旋转误差取 Log(R_meas*R_pred^T)；
/// 位置误差为 t_c1w-R_c1w*R_c2w^T*t_c2w-t_meas；拼接顺序 [旋转,平移]。
class EdgeRelativePose
    : public g2o::BaseBinaryEdge<6, Vector6d, VertexVIPose, VertexVIPose> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeRelativePose();

  /// 本类未实现文件反序列化，调用返回 false。
  virtual bool read(std::istream& is){return false;}
  /// 本类未实现文件序列化，调用返回 false。
  virtual bool write(std::ostream& os) const{return false;}
  /// g2o 在迭代/误差检查时调用，按当前顶点估计更新 _error；不直接更新顶点。
  void computeError();

  Eigen::Matrix3d Rc1c2;
  Eigen::Vector3d tc1c2;
};

#endif  // EDGE_RELATIVE_POSE_H_


#ifndef OPTIMIZATION_3D_TYPES_H_
#define OPTIMIZATION_3D_TYPES_H_

#include <istream>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include "utils.h"
#include "imu.h"

// 这些 struct 是建图前的数据描述，不继承 g2o 顶点/边；真正的图对象在优化函数里创建。
// fixed 表示状态固定；inlier 是观测的内外点标记；id_camera 用于选择相机模型。
// pixel_sigma 是观测权重相关字段，实际是否采用以及如何换算以各建图函数为准。
/// 相机位姿输入/输出：R=Rwc，p=twc（相机在世界中的位置），id_camera 索引 camera_list。
struct Pose3d {
  bool fixed;
  int id_camera;
  Eigen::Vector3d p;
  Eigen::Matrix3d R;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Pose3d() {}
  Pose3d& operator =(Pose3d& other){
		fixed = other.fixed;
		id_camera = other.id_camera;
		p = other.p;
		R = other.R;
		return *this;
	}
};
typedef std::map<int, Pose3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int, Pose3d>>> MapOfPoses;


/// 世界系三维地图点；fixed 决定在 BA 中是否更新它。
struct Position3d{
  bool fixed;
  Eigen::Vector3d p;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Position3d() {}
  Position3d& operator =(Position3d& other){
		fixed = other.fixed;
		p = other.p;
		return *this;
	}
};
typedef std::map<int, Position3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int, Position3d>>> MapOfPoints3d;


/// 单目点观测：通过 id_pose/id_point 连接帧与地图点，keypoint=[uL,vL]。
struct MonoPointConstraint {
  int id_pose;
  int id_point;
  int id_camera;
  bool inlier;

  // x_left, y_left
  Eigen::Vector2d keypoint;  
  double pixel_sigma;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MonoPointConstraint() {}
  MonoPointConstraint& operator =(MonoPointConstraint& other){
		id_pose = other.id_pose;
		id_point = other.id_point;
		id_camera = other.id_camera;
		inlier = other.inlier;
		keypoint = other.keypoint;
		pixel_sigma = other.pixel_sigma;
		return *this;
	}
};
typedef std::shared_ptr<MonoPointConstraint> MonoPointConstraintPtr;
typedef std::vector<MonoPointConstraintPtr> VectorOfMonoPointConstraints;


/// 双目点观测：keypoint=[uL,vL,uR]；inlier 由优化后外点判断更新。
struct StereoPointConstraint {
  int id_pose;
  int id_point;
  int id_camera;
  bool inlier;

  // x_left, y_left, x_right
  Eigen::Vector3d keypoint;  
  double pixel_sigma;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StereoPointConstraint() {}
  StereoPointConstraint& operator =(StereoPointConstraint& other){
		id_pose = other.id_pose;
		id_point = other.id_point;
		id_camera = other.id_camera;
		inlier = other.inlier;
		keypoint = other.keypoint;
		pixel_sigma = other.pixel_sigma;
		return *this;
	}
};
typedef std::shared_ptr<StereoPointConstraint> StereoPointConstraintPtr;
typedef std::vector<StereoPointConstraintPtr> VectorOfStereoPointConstraints;


/// 世界系三维直线（Plucker 表达）；fixed 决定是否更新。
struct Line3d{
  bool fixed;
  g2o::Line3D line_3d;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Line3d() {}
  Line3d& operator =(Line3d& other){
		fixed = other.fixed;
		line_3d = other.line_3d;
		return *this;
	}
};
typedef std::map<int, Line3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int, Line3d>>> MapOfLine3d;


/// 单目线观测：line_2d=[u1,v1,u2,v2]，连接 id_pose 和 id_line。
struct MonoLineConstraint {
  int id_pose;
  int id_line;
  int id_camera;
  bool inlier;

  // x_left, y_left
  Eigen::Vector4d line_2d;  
  double pixel_sigma;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MonoLineConstraint() {}
  MonoLineConstraint& operator =(MonoLineConstraint& other){
		id_pose = other.id_pose;
		id_line = other.id_line;
		id_camera = other.id_camera;
		inlier = other.inlier;
		line_2d = other.line_2d;
		pixel_sigma = other.pixel_sigma;
		return *this;
	}
};
typedef std::shared_ptr<MonoLineConstraint> MonoLineConstraintPtr;
typedef std::vector<MonoLineConstraintPtr> VectorOfMonoLineConstraints;


/// 双目线观测：line_2d=[uL1,vL1,uL2,vL2,uR1,vR1,uR2,vR2]。
struct StereoLineConstraint {
  int id_pose;
  int id_line;
  int id_camera;
  bool inlier;

  Vector8d line_2d;  
  double pixel_sigma;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StereoLineConstraint() {}
  StereoLineConstraint& operator =(StereoLineConstraint& other){
		id_pose = other.id_pose;
		id_line = other.id_line;
		id_camera = other.id_camera;
		inlier = other.inlier;
		line_2d = other.line_2d;
		pixel_sigma = other.pixel_sigma;
		return *this;
	}
};
typedef std::shared_ptr<StereoLineConstraint> StereoLineConstraintPtr;
typedef std::vector<StereoLineConstraintPtr> VectorOfStereoLineConstraints;


// for imu
/// 帧对应的 IMU 世界系速度；容器键与该帧 poses 的键相同。
struct Velocity{
  bool fixed;
  Eigen::Vector3d velocity;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Velocity() {}
  Velocity& operator =(Velocity& other){
		fixed = other.fixed;
		velocity = other.velocity;
		return *this;
	}
};
typedef std::map<int, Velocity, std::less<int>, Eigen::aligned_allocator<std::pair<const int, Velocity>>> MapOfVelocity;


/// 帧对应的 IMU 陀螺仪/加速度计偏置；fixed 控制两个偏置顶点。
struct Bias{
  bool fixed;
  Eigen::Vector3d gyr_bias;
  Eigen::Vector3d acc_bias;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Bias() {}
  Bias& operator =(Bias& other){
		fixed = other.fixed;
		gyr_bias = other.gyr_bias;
		acc_bias = other.acc_bias;
		return *this;
	}
};
typedef std::map<int, Bias, std::less<int>, Eigen::aligned_allocator<std::pair<const int, Bias>>> MapOfBias;


/// id_pose1→id_pose2 的时间段预积分数据；建图后形成 IMU 边及必要的偏置差分边。
struct ImuConstraint{
  int id_pose1;
  int id_pose2;
  int id_camera1;
  int id_camera2;
  PreinterationPtr preinteration;
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuConstraint() {}
  ImuConstraint& operator =(ImuConstraint& other){
		id_pose1 = other.id_pose1;
		id_pose2 = other.id_pose2;
		id_camera1 = other.id_camera1;
		id_camera2 = other.id_camera2;
		preinteration = other.preinteration;
		return *this;
	}
};
typedef std::shared_ptr<ImuConstraint> IMUConstraintPtr;
typedef std::vector<IMUConstraintPtr> VectorOfIMUConstraints;

/// 两帧相对位姿测量 T_c1_c2（相机 2→相机 1），供位姿图优化使用。
struct RelativePoseConstraint{
  int id_pose1;
  int id_pose2;
  int id_camera1;
  int id_camera2;

  Eigen::Matrix3d Rc1c2;
  Eigen::Vector3d tc1c2;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RelativePoseConstraint() {}
  RelativePoseConstraint& operator =(RelativePoseConstraint& other){
		id_pose1 = other.id_pose1;
		id_pose2 = other.id_pose2;
		id_camera1 = other.id_camera1;
		id_camera2 = other.id_camera2;
		Rc1c2 = other.Rc1c2;
		tc1c2 = other.tc1c2;
		return *this;
	}
};
typedef std::shared_ptr<RelativePoseConstraint> RelativePoseConstraintPtr;
typedef std::vector<RelativePoseConstraintPtr> VectorOfRelativePoseConstraints;

#endif  // OPTIMIZATION_3D_TYPES_H_

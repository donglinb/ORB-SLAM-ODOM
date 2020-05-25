#ifndef ODOM_G2O_TYPE_H_
#define ODOM_G2O_TYPE_H_

// #include "Frame.h"
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"


#include <eigen3/Eigen/Core>
#include "se3Types/Sophus/se3.h"
#include "se3Types/Sophus/so3.h"

namespace ORB_SLAM2
{

using Sophus::SE3;
using Sophus::SO3;
using namespace Eigen;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

 class VertexSE3LieAlgebra : public g2o::BaseVertex<6, SE3>
 {
 public:
	 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
     bool read(std::istream& is);
	 bool write(std::ostream& os) const;
	 virtual void setToOriginImpl()
	 {
		 _estimate = Sophus::SE3();
	 }
	 virtual void oplusImpl(const double* update)
	 {
		 Sophus::SE3 up(
			 Sophus::SO3(update[3], update[4], update[5]),
			 Eigen::Vector3d(update[0], update[1], update[2])
		 );
		 _estimate = up * _estimate;
	 }
 };

 class EdgeSE3LieAlgebra : public g2o::BaseBinaryEdge<6, SE3, VertexSE3LieAlgebra, VertexSE3LieAlgebra>
 {
 public:
	 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	 bool read(std::istream& is);
	 bool write(std::ostream& os) const;
	 virtual void computeError()
	 {
		 Sophus::SE3 v1 = (static_cast<VertexSE3LieAlgebra*> (_vertices[0]))->estimate();
		 Sophus::SE3 v2 = (static_cast<VertexSE3LieAlgebra*> (_vertices[1]))->estimate();
		//  _error = (_measurement.inverse()*v1.inverse()*v2).log();
		 _error = (_measurement.inverse()*v1*v2.inverse()).log();
	 }
	 virtual void linearizeOplus();
	 Matrix6d JRInv(SE3 e);
 };

 class EdgeProjectXYZ2UVLieAlgebra :public g2o::BaseBinaryEdge<2, Vector2d, VertexSE3LieAlgebra, g2o::VertexSBAPointXYZ>
 {
 public:
	 EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	 virtual bool read(std::istream& is) { return false; }
	 virtual bool write(std::ostream& os) const { return false; }
	 virtual void computeError()
	 {
		 Sophus::SE3 pose = (static_cast<VertexSE3LieAlgebra*> (_vertices[0]))->estimate();
		 Vector3d point = (static_cast<g2o::VertexSBAPointXYZ*> (_vertices[1]))->estimate();

		 Vector3d p = pose.rotation_matrix()*point + pose.translation();
		 _error = _measurement - cam_project(p);
	 }
	 virtual void linearizeOplus();

	 Matrix3d skew(Vector3d phi);
	 Vector2d cam_project(const Vector3d & trans_xyz) const;
	 bool isDepthPositive();
	 
	 double fx, fy, cx, cy;
 };

}  //namespace ORB_SLAM2

#endif  //ODOM_G2O_TYPE_H_

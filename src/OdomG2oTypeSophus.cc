#include"OdomG2oTypeSophus.h"

namespace ORB_SLAM2
{

 bool VertexSE3LieAlgebra::read(std::istream& is)
 {
	 double data[7];
	 for (int i = 0; i < 7; i++)
		 is >> data[i];
	 setEstimate(SE3(
		 Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
		 Eigen::Vector3d(data[0], data[1], data[2])
	 ));
	 return true;
 }
 bool VertexSE3LieAlgebra::write(std::ostream& os) const
 {
	 os << id() << " ";
	 Eigen::Quaterniond q = _estimate.unit_quaternion();
	 os << _estimate.translation().transpose() << " ";
	 os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << std::endl;
	 return true;
 }


 bool EdgeSE3LieAlgebra::read(std::istream& is)
 {
	 double data[7];
	 for (int i = 0; i < 7; i++)
		 is >> data[i];
	 Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
	 q.normalize();
	 setMeasurement(
		 Sophus::SE3(q, Eigen::Vector3d(data[0], data[1], data[2]))
	 );
	 for (int i = 0; i < information().rows() && is.good(); i++)
		 for (int j = i; j < information().cols() && is.good(); j++)
		 {
			 is >> information() (i, j);
			 if (i != j)
				 information() (j, i) = information() (i, j);
		 }
	 return true;
 }
 bool  EdgeSE3LieAlgebra::write(std::ostream& os) const
 {
	 VertexSE3LieAlgebra* v1 = static_cast<VertexSE3LieAlgebra*> (_vertices[0]);
	 VertexSE3LieAlgebra* v2 = static_cast<VertexSE3LieAlgebra*> (_vertices[1]);
	 os << v1->id() << " " << v2->id() << " ";
	 SE3 m = _measurement;
	 Eigen::Quaterniond q = m.unit_quaternion();
	 os << m.translation().transpose() << " ";
	 os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << " ";
	 // information matrix 
	 for (int i = 0; i < information().rows(); i++)
		 for (int j = i; j < information().cols(); j++)
		 {
			 os << information() (i, j) << " ";
		 }
	 os << std::endl;
	 return true;
 }
 Matrix6d EdgeSE3LieAlgebra::JRInv(SE3 e)
 {
	 Matrix6d J;
	 J.block(0, 0, 3, 3) = SO3::hat(e.so3().log());
	 J.block(0, 3, 3, 3) = SO3::hat(e.translation());
	 J.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero(3, 3);
	 J.block(3, 3, 3, 3) = SO3::hat(e.so3().log());
	 J = J * 0.5 + Matrix6d::Identity();
	 return J;
 }
void EdgeSE3LieAlgebra::linearizeOplus()
 {
	 Sophus::SE3 v1 = (static_cast<VertexSE3LieAlgebra*> (_vertices[0]))->estimate();
	 Sophus::SE3 v2 = (static_cast<VertexSE3LieAlgebra*> (_vertices[1]))->estimate();
	 Matrix6d J = JRInv(SE3::exp(_error));

	//  _jacobianOplusXi = -J * v2.inverse().Adj();
	//  _jacobianOplusXj = J * v2.inverse().Adj();
	_jacobianOplusXi = J*v2.Adj()*v1.inverse().Adj();
	_jacobianOplusXj = -J;
 }


void EdgeProjectXYZ2UVLieAlgebra::linearizeOplus()
{
	Sophus::SE3 pose = (static_cast<VertexSE3LieAlgebra*> (_vertices[0]))->estimate();
	Vector3d point = (static_cast<g2o::VertexSBAPointXYZ*> (_vertices[1]))->estimate();
	Vector3d p = pose.rotation_matrix()*point + pose.translation();

	double x = p[0], y = p[1], z = p[2];
	double z2 = z * z;

	Matrix<double, 2, 3> jacobian_e_p;
	jacobian_e_p << fx / z, 0, -fx * x / z2,
		0, fy / z, -fy * y / z2;
	jacobian_e_p = -jacobian_e_p;

	Matrix<double, 3, 6> jacobian_p_ksi;
	jacobian_p_ksi << Matrix3d::Identity(), -skew(p);

	_jacobianOplusXi = jacobian_e_p * jacobian_p_ksi;
	_jacobianOplusXj = jacobian_e_p * pose.rotation_matrix();
}
Matrix3d EdgeProjectXYZ2UVLieAlgebra::skew(Vector3d phi)
{
	Matrix3d Phi;
	Phi << 0, -phi[2], phi[1],
		phi[2], 0, -phi[0],
		-phi[1], phi[0], 0;
	return Phi;
}
Vector2d EdgeProjectXYZ2UVLieAlgebra::cam_project(const Vector3d & trans_xyz) const
{
	Vector2d res;
	res[0] = fx * trans_xyz[0] / trans_xyz[2] + cx;
	res[1] = fy * trans_xyz[1] / trans_xyz[2] + cy;
	return res;
}
bool EdgeProjectXYZ2UVLieAlgebra::isDepthPositive()
{
	Sophus::SE3 pose = (static_cast<VertexSE3LieAlgebra*> (_vertices[0]))->estimate();
	Vector3d point = (static_cast<g2o::VertexSBAPointXYZ*> (_vertices[1]))->estimate();
	Vector3d p = pose.rotation_matrix()*point + pose.translation();

	return (p[2] > 0.0);
}

}  //namespace ORB_SLAM2
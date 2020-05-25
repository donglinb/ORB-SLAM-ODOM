#ifndef _DQUAT2MAT_H_
#define _DQUAT2MAT_H_
#include <Eigen/Core>

namespace g2o {
  namespace internal {

    void compute_dq_dR ( Eigen::Matrix<double, 3 , 9, Eigen::ColMajor>&  dq_dR , const double&  r11 , const double&  r21 , const double&  r31 , const double&  r12 , const double&  r22 , const double&  r32 , const double&  r13 , const double&  r23 , const double&  r33 );

    void compute_dR_dq ( Eigen::Matrix<double, 9 , 3, Eigen::ColMajor>&  dR_dq , const double&  qx , const double&  qy , const double&  qz , const double&  qw ) ;
  }
}
#endif

// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#ifndef G2O_VERTEX_SE3_QUAT_H
#define G2O_VERTEX_SE3_QUAT_H

#include "../core/base_vertex.h"
#include "../core/hyper_graph_action.h"
#include "se3quat.h"

namespace g2o {

/**
 * \brief 3D pose Vertex, (x,y,z,qw,qx,qy,qz)
 * the parameterization for the increments constructed is a 6d vector
 * (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
 */
class VertexSE3 : public BaseVertex<6, SE3Quat>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexSE3();

    virtual void setToOriginImpl() {
      _estimate = SE3Quat();
    }

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;


    virtual bool setEstimateDataImpl(const double* est){
      Eigen::Map<const Vector7d> v(est);
      _estimate.fromVector(v);
      return true;
    }

    virtual bool getEstimateData(double* est) const{
      Eigen::Map<Vector7d> v(est);
      v=_estimate.toVector();
      return true;
    }

    virtual int estimateDimension() const {
      return 7;
    }

    virtual bool setMinimalEstimateDataImpl(const double* est){
      Eigen::Map<const Vector6d> v(est);
      _estimate.fromMinimalVector(v);
      return true;
    }

    virtual bool getMinimalEstimateData(double* est) const{
      Eigen::Map<Vector6d> v(est);
      v = _estimate.toMinimalVector();
      return true;
    }

    virtual int minimalEstimateDimension() const {
      return 6;
    }

    virtual void oplusImpl(const double* update)
    {
      Eigen::Map<const Vector6d> v(update);
      SE3Quat increment(v);
      _estimate *= increment;
    }
};

  class VertexSE3WriteGnuplotAction: public WriteGnuplotAction {
  public:
    VertexSE3WriteGnuplotAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_ );
  };

#ifdef G2O_HAVE_OPENGL
  class VertexSE3DrawAction: public DrawAction{
  public:
    VertexSE3DrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_ );
    HyperGraphElementAction* _cacheDrawActions;
  protected:
    virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
    FloatProperty* _triangleX, *_triangleY;
  };
#endif

} // end namespace

#endif

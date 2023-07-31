// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
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

#ifndef G2O_EDGE_IMU_H_
#define G2O_EDGE_IMU_H_

#include "g2o/core/base_edge.h"
#include "g2o/core/base_fixed_sized_edge.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o_types_vio_api.h"
#include "vertex_imu_bias.h"
#include "vertex_speed.h"

namespace g2o {

struct ImuMeasurementSE3 {
  Quaternion rotation;
  Vector3 positionDiff;
  double deltaT;
};

/**
 * \brief Edge between two 3D pose vertices
 *
 * The transformation between the two SE3 vertices using measured IMU data and
 * the previous speed. Be sure to use edges for the speeds too.
 */
class G2O_TYPES_VIO_API EdgeImuMeasurement
    : public BaseFixedSizedEdge<6, ImuMeasurementSE3, VertexSE3, VertexSE3,
                                VertexSpeed, VertexImuBias> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeImuMeasurement();
  virtual bool read(std::istream& is) override;
  virtual bool write(std::ostream& os) const override;

  virtual void computeError() override;

  virtual bool setMeasurementData(const double* d) override {
    Eigen::Map<const Vector4> r(d + 3);
    Eigen::Map<const Vector3> p(d);
    _measurement.rotation.coeffs() = r;
    _measurement.positionDiff = p;
    _measurement.deltaT = d[7];
    return true;
  }

  virtual bool getMeasurementData(double* d) const override {
    Eigen::Map<Vector4> r(d + 3);
    Eigen::Map<Vector3> p(d);
    r = _measurement.rotation.coeffs();
    p = _measurement.positionDiff;
    d[7] = _measurement.deltaT;
    return true;
  }

  virtual int measurementDimension() const override { return 8; }

  Isometry3 pseudoMeasurement(const Isometry3& from) const;

 protected:
};

/**
 * \brief Output the pose-pose constraint to Gnuplot data file
 */
class G2O_TYPES_VIO_API EdgeImuMeasurementWriteGnuplotAction
    : public WriteGnuplotAction {
 public:
  EdgeImuMeasurementWriteGnuplotAction();
  virtual HyperGraphElementAction* operator()(
      HyperGraph::HyperGraphElement* element,
      HyperGraphElementAction::Parameters* params_);
};

#ifdef G2O_HAVE_OPENGL
/**
 * \brief Visualize a 3D pose-pose constraint
 */
class G2O_TYPES_VIO_API EdgeImuMeasurementDrawAction : public DrawAction {
 public:
  EdgeImuMeasurementDrawAction();
  virtual HyperGraphElementAction* operator()(
      HyperGraph::HyperGraphElement* element,
      HyperGraphElementAction::Parameters* params_);
};
#endif

}  // namespace g2o
#endif

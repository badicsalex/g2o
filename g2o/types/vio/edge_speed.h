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

#ifndef G2O_EDGE_SPEED_H
#define G2O_EDGE_SPEED_H

#include "g2o/config.h"
#include "g2o/core/base_fixed_sized_edge.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o_types_vio_api.h"
#include "vertex_imu_bias.h"
#include "vertex_speed.h"

namespace g2o {

struct ImuMeasurementSpeed {
  Vector3 speedDiff;
  double deltaT;
  Matrix3 accBiasCovariance;
};
/**
 * \brief Edge between two 3D speed vertices, used for VIO
 *
 * Speed difference (in sensor space) between two speed nodes.
 * First 3 elements of the measurement vector is the speed, the 4th is delta T,
 * used for gravity calculation.
 */
class G2O_TYPES_VIO_API EdgeSpeed
    : public BaseFixedSizedEdge<3, ImuMeasurementSpeed, VertexSpeed,
                                VertexSpeed, VertexSE3, VertexImuBias> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSpeed();

  void computeError() {
    const Vector3& from =
        static_cast<const VertexSpeed*>(_vertices[0])->estimate();
    const Vector3& to =
        static_cast<const VertexSpeed*>(_vertices[1])->estimate();
    const Matrix3& rotation =
        static_cast<const VertexSE3*>(_vertices[2])->estimate().rotation();
    const Vector3& acc_bias =
        static_cast<const VertexImuBias*>(_vertices[3])->estimate().tail<3>();
    _error =
        (to - from) - (rotation * (_measurement.speedDiff +
                                   _measurement.accBiasCovariance * acc_bias) +
                       Vector3(0.0, -9.81, 0.0) * _measurement.deltaT);
  }
  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

  virtual bool setMeasurementData(const double* d) {
    Eigen::Map<const Vector3> sd(d);
    _measurement.speedDiff = sd;

    _measurement.deltaT = d[3];

    Eigen::Map<const Matrix3> cvs(d + 4);
    _measurement.accBiasCovariance = cvs;
    return true;
  }

  virtual bool getMeasurementData(double* d) const {
    Eigen::Map<Vector3> sd(d);
    sd = _measurement.speedDiff;

    d[3] = _measurement.deltaT;

    Eigen::Map<Matrix3> cvs(d + 3);
    cvs = _measurement.accBiasCovariance;

    return true;
  }

  virtual int measurementDimension() const { return 3 + 1 + 9; }
};

}  // namespace g2o

#endif

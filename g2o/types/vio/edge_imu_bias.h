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

#ifndef G2O_EDGE_IMU_BIAS_H
#define G2O_EDGE_IMU_BIAS_H

#include "g2o/config.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o_types_vio_api.h"
#include "vertex_imu_bias.h"

namespace g2o {

/**
 * \brief Edge between two 3D imu_bias vertices, used for VIO
 *
 * ImuBias difference (in sensor space) between two imu_bias nodes.
 * First 3 elements of the measurement vector is the imu_bias, the 4th is delta
 * T, used for gravity calculation.
 */
class G2O_TYPES_VIO_API EdgeImuBias
    : public BaseBinaryEdge<6, float, VertexImuBias, VertexImuBias> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeImuBias();

  void computeError() {
    const Vector6& from =
        static_cast<const VertexImuBias*>(_vertices[0])->estimate();
    const Vector6& to =
        static_cast<const VertexImuBias*>(_vertices[1])->estimate();
    _error = (to - from);
  }
  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

  virtual int measurementDimension() const { return 0; }
};

}  // namespace g2o

#endif

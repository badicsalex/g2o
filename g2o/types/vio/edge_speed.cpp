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

#include "edge_speed.h"

#include "g2o/core/io_helper.h"
#include "isometry3d_gradients.h"

namespace g2o {

EdgeSpeed::EdgeSpeed()
    : BaseFixedSizedEdge<3, ImuMeasurementSpeed, VertexSpeed, VertexSpeed,
                         VertexSE3, VertexImuBias>() {
  _information.setIdentity();
  _error.setZero();
}

bool EdgeSpeed::read(std::istream& is) {
  internal::readVector(is, _measurement.speedDiff);
  is >> _measurement.deltaT;
  for (int i = 0; i < 3 && is.good(); i++)
    for (int j = 0; j < 3 && is.good(); j++)
      is >> _measurement.accBiasCovariance(i, j);
  return readInformationMatrix(is);
}

bool EdgeSpeed::write(std::ostream& os) const {
  internal::writeVector(os, _measurement.speedDiff);
  os << _measurement.deltaT;
  for (int i = 0; i < 3 && os.good(); i++)
    for (int j = 0; j < 3 && os.good(); j++)
      os << _measurement.accBiasCovariance(i, j);
  return writeInformationMatrix(os);
}

}  // namespace g2o

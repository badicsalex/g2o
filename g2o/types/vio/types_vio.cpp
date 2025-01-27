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

#include "types_vio.h"

#include <iostream>
#include <typeinfo>

#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

namespace g2o {

G2O_REGISTER_TYPE_GROUP(vio);

G2O_REGISTER_TYPE(VERTEX_SPEED, VertexSpeed);
G2O_REGISTER_TYPE(VERTEX_IMU_BIAS, VertexImuBias);
G2O_REGISTER_TYPE(EDGE_IMU, EdgeImuMeasurement);
G2O_REGISTER_TYPE(EDGE_SPEED, EdgeSpeed);
G2O_REGISTER_TYPE(EDGE_IMU_BIAS, EdgeImuBias);

#ifdef G2O_HAVE_OPENGL
G2O_REGISTER_ACTION(EdgeImuMeasurementDrawAction);
#endif

G2O_ATTRIBUTE_CONSTRUCTOR(init_vio_types) {
  static bool initialized = false;
  if (initialized) return;
  initialized = true;

/*#ifdef G2O_HAVE_OPENGL
  HyperGraphActionLibrary* actionLib = HyperGraphActionLibrary::instance();
  HyperGraphElementAction::HyperGraphElementActionPtr vertexse3eulerdraw(
      new g2o::VertexSE3DrawAction);
  vertexse3eulerdraw->setTypeName(typeid(VertexSE3Euler).name());
  actionLib->registerAction(vertexse3eulerdraw);

  HyperGraphElementAction::HyperGraphElementActionPtr edgese3eulerdraw(
      new g2o::EdgeSE3DrawAction);
  edgese3eulerdraw->setTypeName(typeid(EdgeSE3Euler).name());
  actionLib->registerAction(edgese3eulerdraw);
#endif*/
}

}  // namespace g2o

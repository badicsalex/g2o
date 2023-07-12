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

#include "edge_imu.h"

#include <Eigen/src/Geometry/Translation.h>

#include "g2o/core/eigen_types.h"
#include "g2o/types/slam3d/isometry3d_gradients.h"
#include "vertex_speed.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_primitives.h"
#include "g2o/stuff/opengl_wrapper.h"
#endif

namespace g2o {

EdgeImuMeasurement::EdgeImuMeasurement()
    : BaseFixedSizedEdge<6, ImuMeasurementSE3, VertexSE3, VertexSE3,
                         VertexSpeed>() {
  information().setIdentity();
}

bool EdgeImuMeasurement::read(std::istream& is) {
  VectorN<8> meas;
  internal::readVector(is, meas);
  // normalize the quaternion to recover numerical precision lost by storing as
  // human readable text
  Vector4::MapType(meas.data() + 3).normalize();
  setMeasurementData(meas.data());
  if (is.bad()) return false;
  readInformationMatrix(is);
  return is.good() || is.eof();
}

bool EdgeImuMeasurement::write(std::ostream& os) const {
  VectorN<8> meas;
  getMeasurementData(meas.data());
  internal::writeVector(os, meas);
  return writeInformationMatrix(os);
}

void EdgeImuMeasurement::computeError() {
  const Isometry3& from = static_cast<VertexSE3*>(_vertices[0])->estimate();
  const Isometry3& to = static_cast<VertexSE3*>(_vertices[1])->estimate();
  const Vector3& speed = static_cast<VertexSpeed*>(_vertices[2])->estimate();

  const Matrix3 fromRot = from.rotation();
  const Matrix3& toRot = to.rotation();

  const Matrix3 rotErr = (fromRot * _measurement.rotation).transpose() * toRot;
  const Vector3 translationErr =
      to.translation() -
      (from.translation() + fromRot * _measurement.positionDiff +
       speed * _measurement.deltaT +
       0.5 * Vector3(0, -9.81, 0) * _measurement.deltaT * _measurement.deltaT);

  _error.block<3, 1>(0, 0) = translationErr;
  _error.block<3, 1>(3, 0) = internal::toCompactQuaternion(rotErr);
}

// TODO: Jacobians for the error above

EdgeImuMeasurementWriteGnuplotAction::EdgeImuMeasurementWriteGnuplotAction()
    : WriteGnuplotAction(typeid(EdgeImuMeasurement).name()) {}

HyperGraphElementAction* EdgeImuMeasurementWriteGnuplotAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters* params_) {
  if (typeid(*element).name() != _typeName) return nullptr;
  WriteGnuplotAction::Parameters* params =
      static_cast<WriteGnuplotAction::Parameters*>(params_);
  if (!params->os) {
    return nullptr;
  }

  EdgeImuMeasurement* e = static_cast<EdgeImuMeasurement*>(element);
  VertexSE3* fromEdge = static_cast<VertexSE3*>(e->vertices()[0]);
  VertexSE3* toEdge = static_cast<VertexSE3*>(e->vertices()[1]);
  Vector6 fromV, toV;
  fromV = internal::toVectorMQT(fromEdge->estimate());
  toV = internal::toVectorMQT(toEdge->estimate());
  for (int i = 0; i < 6; i++) {
    *(params->os) << fromV[i] << " ";
  }
  for (int i = 0; i < 6; i++) {
    *(params->os) << toV[i] << " ";
  }
  *(params->os) << std::endl;
  return this;
}

#ifdef G2O_HAVE_OPENGL
EdgeImuMeasurementDrawAction::EdgeImuMeasurementDrawAction()
    : DrawAction(typeid(EdgeImuMeasurement).name()) {}

HyperGraphElementAction* EdgeImuMeasurementDrawAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters* params_) {
  if (typeid(*element).name() != _typeName) return nullptr;
  refreshPropertyPtrs(params_);
  if (!_previousParams) return this;

  if (_show && !_show->value()) return this;

  EdgeImuMeasurement* e = static_cast<EdgeImuMeasurement*>(element);
  VertexSE3* fromEdge = static_cast<VertexSE3*>(e->vertices()[0]);
  VertexSE3* toEdge = static_cast<VertexSE3*>(e->vertices()[1]);
  if (!fromEdge || !toEdge) return this;
  glColor3f(POSE_EDGE_COLOR);
  glPushAttrib(GL_ENABLE_BIT);
  glDisable(GL_LIGHTING);
  glBegin(GL_LINES);
  glVertex3f((float)fromEdge->estimate().translation().x(),
             (float)fromEdge->estimate().translation().y(),
             (float)fromEdge->estimate().translation().z());
  glVertex3f((float)toEdge->estimate().translation().x(),
             (float)toEdge->estimate().translation().y(),
             (float)toEdge->estimate().translation().z());
  glEnd();
  glPopAttrib();
  return this;
}
#endif

}  // namespace g2o

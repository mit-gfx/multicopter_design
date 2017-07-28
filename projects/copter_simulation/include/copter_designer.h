// @ Copyright 2016 Massachusetts Institute of Technology.
// 
// This program is free software; you can redistribute it and / or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
// MA 02110-1301, USA.
#ifndef _COPTER_SIMULATION_COPTER_DESIGNER_H_
#define _COPTER_SIMULATION_COPTER_DESIGNER_H_

#include <string>
#include <vector>
#include "Eigen/Dense"
#include "battery.h"
#include "design_problem.h"
#include "parametrized_connector.h"
#include "parametrized_rotor.h"
#include "parametrized_tube.h"
#include "rotor.h"
#include "tri_rigid_body.h"
#include "tri_rigid_body_union.h"

namespace tinyxml2 {
class XMLElement;
}

namespace copter_simulation {

enum CopterRigidComponentName {
  kPlate = 0, kBattery, kElectronics, kTube, kRoundConnector,
  kMotor, kCopterRigidComponentNameNum
};

struct CopterReferenceRigidComponentInfo {
public:
  CopterReferenceRigidComponentInfo()
    : reference_rigid_body(),
    model_to_reference_translation(0, 0, 0),
    model_to_reference_rotation(Eigen::Matrix3d::Identity()),
    model_to_reference_scale(1, 1, 1),
    type(kCopterRigidComponentNameNum), index(-1) {}
  CopterReferenceRigidComponentInfo(const TriRigidBody& model_rigid_body,
    const Eigen::Vector3d& translation, const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& scale, const CopterRigidComponentName type,
    const int index) : reference_rigid_body(model_rigid_body),
    model_to_reference_translation(translation),
    model_to_reference_rotation(rotation),
    model_to_reference_scale(scale),
    type(type), index(index) {
    reference_rigid_body.Scale(model_to_reference_scale);
    reference_rigid_body.Rotate(model_to_reference_rotation);
    reference_rigid_body.Translate(model_to_reference_translation);
  }

  TriRigidBody reference_rigid_body;
  Eigen::Vector3d model_to_reference_translation;
  Eigen::Matrix3d model_to_reference_rotation;
  Eigen::Vector3d model_to_reference_scale;
  CopterRigidComponentName type;
  int index;
};

class CopterDesigner {
public:
  CopterDesigner();

  void Initialize(const std::string& xml_file);

  const int NumOfCopterRigidComponent() const {
    return static_cast<int>(reference_rigid_components_.size());
  }
  const CopterReferenceRigidComponentInfo&
  ReferenceRigidComponent(const int i) const {
    return reference_rigid_components_[i];
  }
  const TriMesh PropellerInModelSpace() const {
    return reference_propeller_;
  }
  const int NumOfRotors() const {
    return static_cast<int>(rotors_.size());
  }
  const int NumOfPropellers() const {
    return NumOfRotors();
  }
  const Eigen::VectorXd GetCurrentFullDesignParameter() const;
  const Eigen::VectorXd GetCurrentReducedDesignParameter() const {
    return reduced_design_parameters_;
  }
  // Do not update if parameter breaks design constraints, and return false.
  const bool SetReducedDesignParameter(const Eigen::VectorXd& parameter);
  const DesignProblem& design_constraints() const {
    return design_constraints_;
  }

  // Functions used for visualization.
  const TriMesh ReferenceRigidMeshInBodySpace(const int i) const {
    assert(i >= 0 && i < NumOfCopterRigidComponent());
    return reference_rigid_components_[i].reference_rigid_body
      .body_space_mesh();
  }
  // Applying it to the function above places the rigid mesh to the design
  // space.
  const Eigen::Matrix4d TransformRigidToDesignSpace(const int i) const;
  // Applying it to PropellerInModelSpace places the i-th propeller to the top
  // of the i-th rotor in the design space.
  const Eigen::Matrix4d TransformPropellerToDesignSpace(const int i) const;
  // Same as above except that the target frame is the body frame of the
  // designed copter.
  const Eigen::Matrix4d TransformRigidToDesignBodySpace(const int i) const;
  const Eigen::Matrix4d TransformPropellerToDesignBodySpace(const int i) const;

  const TriRigidBodyUnion& GetCurrentCopterDesign() const {
    return copter_body_;
  }
  const std::vector<Rotor>& GetCurrentRotorDesign() const { return rotors_; }
  const Battery& battery() const { return battery_; }

  void GetRigidComponentLocationInfo(const CopterRigidComponentName name,
    int& start, int& length) const;

  const std::unordered_map<std::string, Eigen::VectorXd>&
    AllParameterNameAndLength() const;

private:
  void ParseLocation(const tinyxml2::XMLElement* node,
    Eigen::Vector3d& xyz, Eigen::Matrix3d& R);
  void ParsePlate(const tinyxml2::XMLElement* node,
    std::string& file_name, double& density, std::vector<Eigen::Vector3d>& xyz,
    std::vector<Eigen::Matrix3d>& R, std::vector<Eigen::Vector2d>& size);
  void ParseTube(const tinyxml2::XMLElement* node,
    std::string& file_name, double& density, std::vector<Eigen::Vector3d>& xyz,
    std::vector<Eigen::Matrix3d>& R, std::vector<double>& length);
  void ParseRoundConnector(const tinyxml2::XMLElement* node,
    std::string& file_name, double& density, std::vector<Eigen::Vector3d>& xyz,
    std::vector<Eigen::Matrix3d>& R);
  void ParseBattery(const tinyxml2::XMLElement* node,
    std::string& file_name, double& density, Eigen::Vector3d& xyz,
    Eigen::Matrix3d& R, double& capacity);
  void ParseMotor(const tinyxml2::XMLElement* node,
    std::string& file_name, double& density, std::string& measurement_file,
    double& propeller_height, std::vector<Eigen::Vector3d>& xyz,
    std::vector<Eigen::Matrix3d>& R, std::vector<bool>& is_ccw,
    std::vector<bool>& flip_propeller);
  void InitializeParametrizedTubes(std::vector<int>& connector_parents);
  void InitializeParametrizedRotors(const std::string& measure_file,
    const double prop_height, const std::vector<bool>& is_ccw,
    const std::vector<bool>& flip_propeller,
    std::vector<int>& motor_parents);
  void InitializeParametrizedConnectors(
    const std::vector<int>& connector_parents,
    const std::vector<int>& motor_parentrs);

  // All components that are rigid attached to the copter are stored here,
  // including plate, battery, electronics, tube, connector, and motor.
  std::vector<CopterReferenceRigidComponentInfo> reference_rigid_components_;
  std::vector<int> rigid_component_offsets_;

  // Propellers spin during flight so we treat them differently. A single prop
  // mesh is stored here, defined in its model space (centering at the origin
  // and spinning in the XoY plane).
  TriMesh reference_propeller_;
  Battery battery_;

  // These parametrized shapes store a reference rigid bodies and a reference
  // parameters. Once given a new parameter, then provide ways to compute a new
  // rigid body, and a transform that maps the reference body to the new body.
  std::vector<ParametrizedTube> reference_tubes_;
  std::vector<ParametrizedConnector> reference_round_connectors_;
  std::vector<ParametrizedRotor> reference_rotors_;

  // Design constraints are built based on the parametrized shapes. Its x0 is
  // the reference parameter defined by the xml file.
  DesignProblem design_constraints_;

  // Once we remove the equality constraints, we get reduce parameters that are
  // bounded only by inequality constraints. By default reduced parameters are
  // all zero, which is equivalent to using the reference parameter.
  // If reduced parameter is nonzero, a new design is proposed. We maintain the
  // transform that from all reference rigid bodies to the new design space,
  // and a rigid copter body, as well as their rotors, are built based on the
  // new design. We guarantee these four data members are always consistent.
  Eigen::VectorXd reduced_design_parameters_;
  std::vector<Eigen::Matrix4d> reference_to_design_transforms_;
  TriRigidBodyUnion copter_body_;
  std::vector<Rotor> rotors_;
};

} // copter_simulation

#endif  // _COPTER_SIMULATION_COPTER_DESIGNER_H_
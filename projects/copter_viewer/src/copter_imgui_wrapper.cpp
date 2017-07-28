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
#include "copter_imgui_wrapper.h"
#include "copter_viewer.h"
#include "imgui.h"

namespace copter_viewer {

CopterImGuiWrapper::CopterImGuiWrapper()
  : parent_viewer_(NULL) {}

void CopterImGuiWrapper::Initialize(CopterViewer* parent_viewer) {
  parent_viewer_ = parent_viewer;
}

void CopterImGuiWrapper::SetupUi() {
  ImGui::SetNextWindowSize(ImVec2(350.0f, 120.0f), ImGuiSetCond_Once);
  ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiSetCond_Once);
  ImGui::Begin("Copter Property");
  DisplayPhysicalProperty();
  ImGui::End();

  ImGui::SetNextWindowSize(ImVec2(350.0f, 240.0f), ImGuiSetCond_Once);
  ImGui::SetNextWindowPos(ImVec2(20.0f, 160.0f), ImGuiSetCond_Once);
  ImGui::Begin("IMU Status");
  if (ImGui::CollapsingHeader("Sensor Data", ImGuiTreeNodeFlags_DefaultOpen)) {
    DisplaySensorData();
  }
  if (ImGui::CollapsingHeader("Ground Truth",
    ImGuiTreeNodeFlags_DefaultOpen)) {
    DisplayGroundTruthData();
  }
  ImGui::End();

  ImGui::SetNextWindowSize(ImVec2(350.0f, 300.0f), ImGuiSetCond_Once);
  ImGui::SetNextWindowPos(ImVec2(20.0f, 420.0f), ImGuiSetCond_Once);
  ImGui::Begin("Propeller Data");
  DisplayRotorData();
  ImGui::End();

  ImGui::SetNextWindowSize(ImVec2(350.0f, 100.0f), ImGuiSetCond_Once);
  ImGui::SetNextWindowPos(ImVec2(20.0f, 740.0f), ImGuiSetCond_Once);
  ImGui::Begin("Battery Status");
  DisplayBatteryStatus();
  ImGui::End();

  ImGui::SetNextWindowSize(ImVec2(550.0f, 520.0f), ImGuiSetCond_Once);
  ImGui::SetNextWindowPos(ImVec2(1030.0f, 20.0f), ImGuiSetCond_Once);
  ImGui::Begin("Design Panel");
  DisplayDesignPanel();
  ImGui::End();
}

void CopterImGuiWrapper::DisplayPhysicalProperty() {
  // Mass and moment of inertia.
  const copter_simulation::RigidBody& copter_body =
    parent_viewer_->copter().copter_body();

  const double mass = copter_body.mass();
  ImGui::BulletText(("Mass (kg): " + std::to_string(mass)).c_str());

  const Eigen::Matrix3d inertia_body = copter_body.inertia_body();
  std::stringstream inertia_str;
  inertia_str << inertia_body;
  ImGui::BulletText(("Moment of Inertia (kgm^2): \n"
    + inertia_str.str()).c_str());
}

void CopterImGuiWrapper::DisplaySensorData() {
  const copter_simulation::InertialSensor& sensor =
    parent_viewer_->copter().inertial_sensor();

  const Eigen::Vector3d pos = sensor.position();
  std::stringstream sensor_str;
  sensor_str.precision(3);
  sensor_str.setf(std::ios::fixed, std::ios::floatfield);
  sensor_str << "Position (m): " << pos.x() << ", "
    << pos.y() << ", " << pos.z();
  ImGui::BulletText(sensor_str.str().c_str());

  const Eigen::Vector3d rpy = copter_simulation::RadianToDegree(sensor.rpy());
  sensor_str.str("");
  sensor_str << "Euler Angle (degree): " << rpy(0) << ", "
    << rpy(1) << ", " << rpy(2);
  ImGui::BulletText(sensor_str.str().c_str());

  const Eigen::Vector3d velocity = sensor.velocity();
  sensor_str.str("");
  sensor_str << "Velocity (m/s): " << velocity.x() << ", "
    << velocity.y() << ", " << velocity.z();
  ImGui::BulletText(sensor_str.str().c_str());

  const Eigen::Vector3d euler_rate =
    copter_simulation::RadianToDegree(sensor.rpy_rate());
  sensor_str.str("");
  sensor_str << "Euler Rate (degree/s): " << euler_rate(0) << ", "
    << euler_rate(1) << ", " << euler_rate(2);
  ImGui::BulletText(sensor_str.str().c_str());
}

void CopterImGuiWrapper::DisplayGroundTruthData() {
  const copter_simulation::RigidBody& copter_body =
    parent_viewer_->copter().copter_body();
  const Eigen::Vector3d pos = copter_body.position();
  std::stringstream ground_truth_str;
  ground_truth_str.precision(3);
  ground_truth_str.setf(std::ios::fixed, std::ios::floatfield);
  ground_truth_str << "Position (m): " << pos.x() << ", "
    << pos.y() << ", " << pos.z();
  ImGui::BulletText(ground_truth_str.str().c_str());

  const Eigen::Vector3d rpy = copter_simulation::RadianToDegree(
    copter_simulation::RotationToRollPitchYaw(copter_body.orientation()));
  ground_truth_str.str("");
  ground_truth_str << "Euler Angle (degree): " << rpy(0) << ", "
    << rpy(1) << ", " << rpy(2);
  ImGui::BulletText(ground_truth_str.str().c_str());

  const Eigen::Vector3d velocity = copter_body.velocity();
  ground_truth_str.str("");
  ground_truth_str << "Velocity (m/s): " << velocity.x() << ", "
    << velocity.y() << ", " << velocity.z();
  ImGui::BulletText(ground_truth_str.str().c_str());

  const Eigen::Vector3d euler_rate = copter_simulation::RadianToDegree(
    copter_simulation::AngularRateToEulerRate(
      copter_simulation::DegreeToRadian(rpy), copter_body.angular_velocity()));
  ground_truth_str.str("");
  ground_truth_str << "Euler Rate (degree/s): " << euler_rate(0) << ", "
    << euler_rate(1) << ", " << euler_rate(2);
  ImGui::BulletText(ground_truth_str.str().c_str());
}

void CopterImGuiWrapper::DisplayRotorData() {
  const copter_simulation::Copter& copter = parent_viewer_->copter();
  const int rotor_num = copter.NumOfRotors();
  for (int i = 0; i < rotor_num; ++i) {
    const copter_simulation::Rotor& rotor = copter.rotor(i);
    std::stringstream rotor_str;
    rotor_str.precision(3);
    rotor_str.setf(std::ios::fixed, std::ios::floatfield);
    rotor_str << "Rotor " << i << ":";
    if (ImGui::CollapsingHeader(rotor_str.str().c_str())) {
      // Position.
      rotor_str.str("");
      const Eigen::Vector3d position = rotor.position();
      rotor_str << "Position (m): " << position.x() << ", "
        << position.y() << ", " << position.z();
      ImGui::BulletText(rotor_str.str().c_str());

      // Direction.
      rotor_str.str("");
      const Eigen::Vector3d direction = rotor.direction();
      rotor_str << "Direction (m): " << direction.x() << ", "
        << direction.y() << ", " << direction.z();
      ImGui::BulletText(rotor_str.str().c_str());
      
      // Spinning.
      rotor_str.str("");
      rotor_str << "Spinning: " << (rotor.is_ccw() ? "CCW" : "CW");
      ImGui::BulletText(rotor_str.str().c_str());

      // Speed (Rad/s).
      rotor_str.str("");
      const double prop_speed = rotor.propeller_speed();
      rotor_str << "Speed (rpm): " << prop_speed * 30.0 / opengl_viewer::kPi;
      ImGui::BulletText(rotor_str.str().c_str());

      // Thrust (N).
      rotor_str.str("");
      rotor_str << "Thrust (N): " << rotor.propeller_thrust_coefficient() *
        prop_speed * prop_speed;
      ImGui::BulletText(rotor_str.str().c_str());
    }
  }
}

void CopterImGuiWrapper::DisplayBatteryStatus() {
  const copter_simulation::Battery& battery =
    parent_viewer_->copter().battery();
  // Decide the progress bar based on the percentage.
  const float percentage = static_cast<float>(battery.Percent());
  ImColor bar_color;
  if (percentage > 0.6) {
    bar_color = ImColor(103, 199, 111);
  } else if (percentage > 0.2) {
    bar_color = ImColor(234, 224, 38);
  } else {
    bar_color = ImColor(220, 34, 38);
  }
  ImGui::PushStyleColor(ImGuiCol_PlotHistogram, bar_color);
  ImGui::ProgressBar(static_cast<float>(battery.Percent()),
    ImVec2(-1.0f, 0.0f));
  ImGui::PopStyleColor();

  const double estimated_minute = battery.EstimateRemainingTime();
  if (estimated_minute >= 0.0) {
    const int minute = static_cast<int>(estimated_minute);
    const int second = static_cast<int>((estimated_minute - minute) * 60.0);
    std::stringstream battery_str;
    // Print minutes.
    if (minute > 1) battery_str << minute << " minutes ";
    else if (minute == 1) battery_str << "1 minute ";
    // Print seconds.
    if (second > 1) battery_str << second << " seconds left";
    else if (second == 1) battery_str << "1 second left";
    else if (minute > 0) battery_str << "left";
    else battery_str << "The battery is empty";
    ImGui::Text(battery_str.str().c_str());
  }
}

void CopterImGuiWrapper::DisplayDesignPanel() {
  if (ImGui::CollapsingHeader("Full Parameters",
    ImGuiTreeNodeFlags_DefaultOpen)) {
    DisplayFullParameters();
  }
  if (ImGui::CollapsingHeader("Reduced Parameters",
    ImGuiTreeNodeFlags_DefaultOpen)) {
    DisplayReducedParameters();
  }
  if (ImGui::CollapsingHeader("Simulator",
    ImGuiTreeNodeFlags_DefaultOpen)) {
    DisplaySimulator();
  }
}

void CopterImGuiWrapper::DisplayReducedParameters() {
  copter_simulation::Copter& copter_instance = parent_viewer_->copter();
  const CopterViewerPhase current_phase = parent_viewer_->phase();

  // A list of reduced parameters.
  const copter_simulation::DesignProblem& problem =
    copter_instance.designer().design_constraints();
  const int bounded_param_num = problem.bounded_parameter_num();
  const int free_param_num = problem.free_parameter_num();

  // Get the current parameters.
  Eigen::VectorXf reduced_param =
    copter_instance.GetReducedDesignParameter().cast<float>();

  // Display constrained variables.
  ImGui::Text("Constrained Parameters:");
  ImGui::SameLine();
  if (ImGui::Button("Reset Reduced Parameters", ImVec2(180, 18))
    && current_phase == kDesignPhase) {
    reduced_param.setZero();
  }
  ImGui::Spacing();

  const Eigen::MatrixXf F = problem.F().cast<float>();
  const Eigen::VectorXf g = problem.g().cast<float>();
  const float inf = static_cast<float>(
    copter_simulation::Constraint::Infinity());
  for (int i = 0; i < bounded_param_num; ++i) {
    const Eigen::VectorXf fi = F.col(i);
    const Eigen::VectorXf gi = g - F * reduced_param.head(bounded_param_num)
      + fi * reduced_param(i);
    // Should guarantee fi * reduced_param(i) <= gi.
    float lower_bd = -inf, upper_bd = inf;
    for (auto j = 0; j < gi.size(); ++j) {
      const float fij = fi(j), gij = gi(j);
      if (fij == 0.0) continue;
      else if (fij > 0.0) {
        const float new_upper = gij / fij;
        if (new_upper < upper_bd) upper_bd = new_upper;
      } else {
        const float new_lower = gij / fij;
        if (new_lower > lower_bd) lower_bd = new_lower;
      }
    }
    // Clamp reduced_param(i) so that it falls between lower and upper bound.
    float new_param = reduced_param(i);
    if (new_param > upper_bd) new_param = upper_bd;
    else if (new_param < lower_bd) new_param = lower_bd;
    reduced_param(i) = new_param;

    std::stringstream bound_str;
    bound_str.precision(3);
    bound_str.setf(std::ios::fixed, std::ios::floatfield);
    bound_str << "(" << lower_bd << ", " << upper_bd << ")";
    if (lower_bd + 1e-3 < upper_bd) {
      ImGui::PushStyleColor(ImGuiCol_FrameBg, ImColor(140, 230, 130, 120));
    } else {
      // Fixed parameters.
      ImGui::PushStyleColor(ImGuiCol_FrameBg, ImColor(210, 140, 150, 120));
    }
    ImGui::SliderFloat(("Param " + std::to_string(i) + " "
      + bound_str.str()).c_str(),
      &reduced_param(i), lower_bd, upper_bd, "%.3f");
    ImGui::PopStyleColor();
  }

  // Display free variables if there is any.
  if (free_param_num) {
    ImGui::Separator();
    ImGui::Text("Free Parameters:");
    for (int i = 0; i < free_param_num; ++i) {
      const int param_id = bounded_param_num + i;
      const float current_value = reduced_param(param_id);
      // TODO: I assume -1000 to +1000 should be large enough.
      ImGui::SliderFloat(("Param " + std::to_string(param_id)).c_str(),
        &reduced_param(param_id), -1000.0f, 1000.0f, "%.3f", 10.0f);
    }
  }

  // If we are in the design phase, send parameters back to the copter.
  if (current_phase == kDesignPhase) {
    copter_instance.SetReducedDesignParameter(reduced_param.cast<double>());
  }
}

void CopterImGuiWrapper::DisplayFullParameters() {
  copter_simulation::Copter& copter_instance = parent_viewer_->copter();
  const CopterViewerPhase current_phase = parent_viewer_->phase();

  // A list of reduced parameters.
  const copter_simulation::CopterDesigner& designer =
    copter_instance.designer();
  const copter_simulation::DesignProblem& problem =
    designer.design_constraints();

  // Get the current parameters.
  Eigen::VectorXf full_param =
    copter_instance.GetFullDesignParameter().cast<float>();

  // Display constrained variables.
  if (ImGui::Button("Reset Full Parameters", ImVec2(180, 18))
    && current_phase == kDesignPhase) {
    full_param = problem.x0().cast<float>();
  }
  ImGui::Spacing();
  const std::unordered_map<std::string, Eigen::VectorXd>&
    all_parameter_info = designer.AllParameterNameAndLength();
  ImGui::PushStyleColor(ImGuiCol_FrameBg, ImColor(140, 230, 130, 120));
  int offset = 0;
  for (const auto& pair : all_parameter_info) {
    const std::string& name = pair.first;
    const int length = static_cast<int>(pair.second.size());
    for (int i = 0; i < length; ++i) {
      // I assume -2.0 to 2.0 is enough for full parameters.
      ImGui::SliderFloat((name + ": " + std::to_string(i)).c_str(),
        &full_param(offset + i), -2.0f, 2.0f, "%.3f");
    }
    offset += length;
  }
  ImGui::PopStyleColor();

  // If we are in the design phase, send parameters back to the copter.
  if (current_phase == kDesignPhase) {
    copter_instance.SetFullDesignParameter(full_param.cast<double>());
  }
}

void CopterImGuiWrapper::DisplaySimulator() {
  copter_simulation::Copter& copter_instance = parent_viewer_->copter();
  const CopterViewerPhase current_phase = parent_viewer_->phase();

  if (current_phase == kDesignPhase) {
    if (ImGui::Button("Simulate", ImVec2(180, 18))) {
      parent_viewer_->set_phase(kSimulationPhase);
      copter_instance.InitializeController();
    }
    ImGui::SameLine();
    ImGui::Text("Press when you finish your design.");
  } else if (current_phase == kSimulationPhase) {
    if (ImGui::Button("Reset", ImVec2(180, 18))) {
      parent_viewer_->set_phase(kDesignPhase);
      copter_instance.SetReducedDesignParameter(
        copter_instance.GetReducedDesignParameter());
      copter_instance.InitializeController();
    }
    ImGui::SameLine();
    ImGui::Text("Press when you want to change your design.");
  } else {
    // Should not happen.
    assert(false);
  }
}

} // copter_viewer
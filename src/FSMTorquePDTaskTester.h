#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/TorquePDJointTask.h>
#include <mc_tasks/TorquePDRelativeCartesianTask.h>
#include "api.h"

struct FSMTorquePDTaskTester_DLLAPI FSMTorquePDTaskTester : public mc_control::fsm::Controller
{
  FSMTorquePDTaskTester(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  std::shared_ptr<mc_tasks::TorquePDJointTask> torquePDTask;
  std::shared_ptr<mc_tasks::TorquePDRelativeCartesianTask> RightHandTask;
  std::shared_ptr<mc_tasks::TorquePDRelativeCartesianTask> LeftHandTask;

  std::pair<sva::PTransformd, Eigen::Vector3d>  createContactAnchor(const mc_rbdyn::Robot & anchorRobot);
private:
  mc_rtc::Configuration config_;
};

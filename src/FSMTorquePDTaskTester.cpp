#include "FSMTorquePDTaskTester.h"
#include <Eigen/src/Core/Matrix.h>

FSMTorquePDTaskTester::FSMTorquePDTaskTester(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  selfCollisionConstraint->setCollisionsDampers(solver(), {1.1, 9.0});
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
      new mc_solver::DynamicsConstraint(
          robots(), 0, {0.1, 0.01, 0.0, 1.8, 70.0}, 0.9, true));
  solver().addConstraintSet(dynamicsConstraint);

  // Remove the default posture task created by the FSM
  solver().removeTask(getPostureTask(robot().name()));

  torquePDTask = std::make_shared<mc_tasks::TorquePDJointTask>(
      solver(), robot().robotIndex(), 100.0, 1);

  solver().addTask(torquePDTask);

  // Print available frames for debugging
  const auto & robot_ = robots().robot(robot().robotIndex());
  mc_rtc::log::info("Available frames:");
  for(const auto & frame : robot_.frames())
  {
    mc_rtc::log::info(" - {}", frame);
  }

  std::string tool_frame = config("frame_name", (std::string) "tool_frame");
  std::string relative_frame = config("relative_frame_name", (std::string) "world");


  torquePDCartesianTask = std::make_shared<mc_tasks::TorquePDCartesianTask>(
      solver(), tool_frame, robot().robotIndex());
  torquePDCartesianTask->setCompensateGravity(true);
  
  solver().addTask(torquePDCartesianTask);

  // torquePDRelativeCartesianTask = std::make_shared<mc_tasks::TorquePDRelativeCartesianTask>(
  //     solver(), tool_frame, relative_frame, robots(), robot().robotIndex());
  // torquePDRelativeCartesianTask->setCompensateGravity(true);
  // solver().addTask(torquePDRelativeCartesianTask);

  datastore().make<std::string>("ControlMode", "Torque");
  datastore().make<std::string>("TorqueMode", "Custom");
  datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr { return postureTask; });

  mc_rtc::log::success("FSMTorquePDTaskTester init done ");
}

bool FSMTorquePDTaskTester::run()
{
  return mc_control::fsm::Controller::run(
        mc_solver::FeedbackType::ClosedLoopIntegrateReal);
}

void FSMTorquePDTaskTester::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}

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

  std::string left_hand_frame = config("left_hand_frame_name", (std::string) "left_hand_link");
  std::string right_hand_frame = config("right_hand_frame_name", (std::string) "right_hand_link");
  // std::string relative_frame = config("relative_frame_name", (std::string) "world");

  LeftHandTask = std::make_shared<mc_tasks::TorquePDRelativeCartesianTask>(
      solver(), left_hand_frame, robots(), robot().robotIndex());
  LeftHandTask->setCompensateGravity(true);
  solver().addTask(LeftHandTask);

  RightHandTask = std::make_shared<mc_tasks::TorquePDRelativeCartesianTask>(
      solver(), right_hand_frame, robots(), robot().robotIndex());
  RightHandTask->setCompensateGravity(true);
  solver().addTask(RightHandTask);

  datastore().make<std::string>("ControlMode", "Torque");
  if(!datastore().has("anchorFrameFunction"))
  {
    datastore().make_call("anchorFrameFunction", [this](const mc_rbdyn::Robot & robot) {return createContactAnchor(robot);});
  }

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

std::pair<sva::PTransformd, Eigen::Vector3d> FSMTorquePDTaskTester::createContactAnchor(const mc_rbdyn::Robot & anchorRobot)
{
  sva::PTransformd X_foot_r = anchorRobot.bodyPosW("right_ankle_link");
  sva::PTransformd X_foot_l = anchorRobot.bodyPosW("left_ankle_link");

  sva::MotionVecd v_foot_r = anchorRobot.bodyVelW("right_ankle_link");
  sva::MotionVecd v_foot_l = anchorRobot.bodyVelW("left_ankle_link");

  auto extTorqueSensor = robot().device<mc_rbdyn::VirtualTorqueSensor>("ExtTorquesVirtSensor");
  // double tau_ext_knee_r = extTorqueSensor.torques()[robot().jointIndexByName("right_knee_joint")];
  // double tau_ext_knee_l = extTorqueSensor.torques()[robot().jointIndexByName("left_knee_joint")];
  double tau_ext_knee_r =  abs(extTorqueSensor.torques()[8+6]);
  double tau_ext_knee_l =  abs(extTorqueSensor.torques()[3+6]);
  if(tau_ext_knee_r + tau_ext_knee_l < 1e-6)
  {
    tau_ext_knee_l = 1;
    tau_ext_knee_r = 1;
  }
  double leftFootRatio = tau_ext_knee_l/(tau_ext_knee_r+tau_ext_knee_l);
                              
  Eigen::VectorXd w_r = X_foot_r.translation();//* tau_ext_knee_r/(tau_ext_knee_r+tau_ext_knee_l);
  Eigen::VectorXd w_l = X_foot_l.translation(); //* tau_ext_knee_l/(tau_ext_knee_r+tau_ext_knee_l);
  Eigen::VectorXd contact_anchor = (w_r * (1 - leftFootRatio) + w_l * leftFootRatio)  ;
  Eigen::VectorXd anchor_vel = (v_foot_r.linear() * (1 - leftFootRatio) + v_foot_l.linear() * leftFootRatio);
  sva::PTransformd contact_anchor_tf(Eigen::Matrix3d::Identity(), contact_anchor); 

  return {contact_anchor_tf, anchor_vel};
}
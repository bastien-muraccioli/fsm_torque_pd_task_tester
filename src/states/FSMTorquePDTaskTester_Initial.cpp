#include "FSMTorquePDTaskTester_Initial.h"

#include "../FSMTorquePDTaskTester.h"

void FSMTorquePDTaskTester_Initial::configure(const mc_rtc::Configuration & config) {}

void FSMTorquePDTaskTester_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<FSMTorquePDTaskTester &>(ctl_);
}

bool FSMTorquePDTaskTester_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<FSMTorquePDTaskTester &>(ctl_);
  // output("OK");
  return false;
}

void FSMTorquePDTaskTester_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<FSMTorquePDTaskTester &>(ctl_);
}

EXPORT_SINGLE_STATE("FSMTorquePDTaskTester_Initial", FSMTorquePDTaskTester_Initial)

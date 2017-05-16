#pragma once
#include <mc_control/mc_controller.h>
#include <mc_control/api.h>

namespace mc_control {

  struct MC_CONTROL_DLLAPI MCHMC2TestController : public MCController {

  public:
    MCHMC2TestController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt);
  };

  SIMPLE_CONTROLLER_CONSTRUCTOR("HMC2Test", mc_control::MCHMC2TestController)
}

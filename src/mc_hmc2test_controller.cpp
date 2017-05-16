#include "mc_hmc2test_controller.h"

namespace mc_control {

  MCHMC2TestController::MCHMC2TestController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
    : MCController(robot_module, dt) {
  }
}

#pragma once
#include <mc_control/mc_controller.h>
#include <mc_control/api.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/OrientationTask.h>

namespace mc_control {

  struct MC_CONTROL_DLLAPI MCHMC2TestController : public MCController {

  public:
    MCHMC2TestController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt);

    virtual bool run() override;
    virtual void reset(const ControllerResetData & reset_data) override;

    std::shared_ptr<mc_tasks::CoMTask> comTask;
    std::shared_ptr<mc_tasks::OrientationTask> orBodyTask;
  };

  SIMPLE_CONTROLLER_CONSTRUCTOR("HMC2Test", mc_control::MCHMC2TestController)
}

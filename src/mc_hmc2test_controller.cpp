#include <mc_rtc/logging.h>
#include "mc_hmc2test_controller.h"

namespace mc_control {

  MCHMC2TestController::MCHMC2TestController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
    : MCController(robot_module, dt) {

    solver().addConstraintSet(contactConstraint);
    solver().addConstraintSet(dynamicsConstraint);

    postureTask->stiffness(30.0);
    postureTask->weight(50.0);
    
    solver().addTask(postureTask.get());

    comTask = std::make_shared<mc_tasks::CoMTask>(robots(), robots().robotIndex(), 50.0, 10.0);
    solver().addTask(comTask);

    orBodyTask = std::make_shared<mc_tasks::OrientationTask>("Body", robots(), 0, 50.0, 10.0);
    orBodyTask->orientation(Eigen::Matrix3d::Identity());
    solver().addTask(orBodyTask);

    solver().setContacts({{robots(), 0, 1, "LeftFoot",  "AllGround"},
                          {robots(), 0, 1, "RightFoot", "AllGround"}});

    //solver().setContacts({});

    LOG_SUCCESS("MCHMC2TestController init done" << this);
  }

  bool MCHMC2TestController::run() {

    bool ret = MCController::run();
    return ret;
  }
  
  void MCHMC2TestController::reset(const ControllerResetData & reset_data) {

    auto comT = rbd::computeCoM(robot().mb(), robot().mbc());
    std::cout << "CoM (first): " << comT.transpose() << std::endl;
    comTask->com(Eigen::Vector3d(0.0, 0.0, comT[2]));
    
    MCController::reset(reset_data);
  }
}

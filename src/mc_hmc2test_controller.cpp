#include <mc_rtc/logging.h>
#include "mc_hmc2test_controller.h"

namespace mc_control {

  MCHMC2TestController::MCHMC2TestController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
    : MCController(robot_module, dt) {

    solver().addConstraintSet(contactConstraint);
    solver().addConstraintSet(kinematicsConstraint);
    solver().addTask(postureTask.get());

    /*
    comTask = std::make_shared<mc_tasks::CoMTask>(robots(), robots().robotIndex(), 5.0, 10.0);
    robot().mbc().q = postureTask->posture();
    auto comT = rbd::computeCoM(robot().mb(), robot().mbc());
    std::cout << comT << std::endl;
    comTask->com(comT);
    solver().addTask(comTask);

    solver().setContacts({{robots(), 0, 1, "LeftFoot",  "AllGround"},
                          {robots(), 0, 1, "RightFoot", "AllGround"}});
    */

    solver().setContacts({});

    head_joint_index = robot().jointIndexByName("HY");

    LOG_SUCCESS("MCHMC2TestController init done" << this);
  }

  bool MCHMC2TestController::run() {

    bool ret = MCController::run();

    if (std::abs(postureTask->posture()[head_joint_index][0] - robot().mbc().q[head_joint_index][0]) < 0.05)
      switch_target();
    
    return ret;
  }
  
  void MCHMC2TestController::reset(const ControllerResetData & reset_data) {
    MCController::reset(reset_data);
    target_left = true;
    switch_target();
  }

  void MCHMC2TestController::switch_target() {

    double target;

    if (target_left)
      target = robot().qu()[head_joint_index][0];
    else
      target = robot().ql()[head_joint_index][0];

    std::vector<std::vector<double>> cur_obj = postureTask->posture();
    cur_obj[head_joint_index][0] = target;
    postureTask->posture(cur_obj);
    target_left = !target_left;
  }
}

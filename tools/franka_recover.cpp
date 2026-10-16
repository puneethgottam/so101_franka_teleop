// Clears a Franka error/Reflex state via libfranka automaticErrorRecovery().
// Does NOT command any motion. Run only while no other FCI client (ROS stack) is connected.
//   usage: franka_recover <robot_ip>
#include <franka/exception.h>
#include <franka/robot.h>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: " << argv[0] << " <robot_ip>\n";
    return 2;
  }
  try {
    franka::Robot robot(argv[1]);
    auto state = robot.readOnce();
    std::cout << "mode before: " << static_cast<int>(state.robot_mode)
              << " (0=Other 1=Idle 2=Move 3=Guiding 4=Reflex 5=UserStopped 6=AutoRecovery)\n"
              << "current errors: " << static_cast<std::string>(state.current_errors) << "\n"
              << "last motion errors: " << static_cast<std::string>(state.last_motion_errors) << "\n";
    robot.automaticErrorRecovery();
    state = robot.readOnce();
    std::cout << "mode after:  " << static_cast<int>(state.robot_mode) << "\n";
    return state.robot_mode == franka::RobotMode::kIdle ? 0 : 1;
  } catch (const franka::Exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}

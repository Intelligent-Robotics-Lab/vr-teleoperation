#include "pepper_vr_controller/RobotControllerApp.h"

int main(int argc, char** argv){
    std::unique_ptr<RobotController> robotController(new RobotController(argc, argv));
    robotController->Init();
    robotController->Run();
    robotController->Shutdown();

    return 0;
}
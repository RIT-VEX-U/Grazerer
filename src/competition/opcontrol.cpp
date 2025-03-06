#include "competition/opcontrol.h"
#include "competition/autonomous.h"
#include "robot-config.h"
#include "vex.h"

void testing();
void konamiCode();
void inputCheck();
void auto__();

/**
 * Main entrypoint for the driver control period
 */

enum INPUTS{
    LEFT,
    RIGHT,
    UP,
    DOWN,
    B,
    A,
    X,
    Y,
    R1,
    R2,
    L1,
    L2,
};
std::vector<INPUTS> inputTracker;
bool runDrive = true;

void opcontrol() {
    // vexDelay(1000);
    // autonomous();
    // return;
    testing();
    wallstakemech_sys.hold = false;
    // intake_sys.conveyor_stop();
    intake_sys.setLight(false);
    wallstakemech_sys.set_setpoint(from_degrees(210));

    ColorSortToggle.pressed([]() { intake_sys.set_color_sort_bool(!intake_sys.get_color_sort_bool()); });

    goal_grabber.pressed([]() { clamper_sys.toggle_clamp(); });

    conveyor_button.pressed([]() {
        intake_sys.intake();
        intake_sys.conveyor_in();
    });
    conveyor_button_rev.pressed([]() {
        intake_sys.outtake();
        intake_sys.conveyor_out();
    });

    conveyor_button.released([]() {
        intake_sys.intake_stop();
        intake_sys.conveyor_stop();
        
    });

    conveyor_button_rev.released([]() {
        intake_sys.intake_stop();
        intake_sys.conveyor_stop();
    });

    wallstake_toggler.pressed([]() {
        wallstakemech_sys.hold = true;
        if (wallstakemech_sys.get_angle().degrees() > 180 || wallstake_motors.velocity(vex::velocityUnits::dps) > 5) {
            wallstakemech_sys.set_setpoint(from_degrees(170));
        } else if (wallstakemech_sys.get_angle().degrees() < 180) {
            wallstakemech_sys.set_setpoint(from_degrees(50));
        }
    });

    wallstake_stow.pressed([]() {
        wallstakemech_sys.hold = true;
        wallstakemech_sys.set_setpoint(from_degrees(200));
    });
    //stupid shit
    inputCheck();

    // ================ INIT ================
    while (true) {
        if (!conveyor_button.pressing() && !conveyor_button_rev.pressing()) {
            intake_sys.intake_stop();
            intake_sys.conveyor_stop();
        }
        konamiCode();

        OdometryBase *odombase = &odom;
        Pose2d pos = odombase->get_position();
        printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x(), pos.y(), pos.rotation().degrees());

        double left = (double)con.Axis3.position() / 100;
        double right = (double)con.Axis2.position() / 100;
        if(runDrive){
            drive_sys.drive_tank(left, right, 1, TankDrive::BrakeType::None);
        }

        vexDelay(20);
    }

    // ================ PERIODIC ================
}

void printInput(){
    for(int i = 0; i <inputTracker.size(); i++){
        switch(inputTracker.at(i)){
            case UP:
                printf("UP, ");
                break;
            case DOWN:
                printf("DOWN, ");
                break;
            case LEFT:
                printf("LEFT, ");
                break;
            case RIGHT:
                printf("RIGHT, ");
                break;
            case A:
                printf("A, ");
                break;
            case B:
                printf("B, ");
                break;
            case X:
                printf("X, ");
                break;
            case Y:
                printf("Y, ");
                break;
            case R1:
                printf("R1, ");
                break;
            case R2:
                printf("R2, ");
                break;
            case L1:
                printf("L1, ");
                break;
            case L2:
                printf("L2, ");
                break;
                default:
                printf("unkown, ");
                break;

        }
    }
    printf("\n");
}

void inputCheck(){
    con.ButtonX.pressed([](){
        inputTracker.push_back(X);
        printInput();
    });
    con.ButtonY.pressed([](){
        inputTracker.push_back(Y);
        printInput();
    });
    con.ButtonA.pressed([](){
        inputTracker.push_back(A);
        printInput();
    });
    con.ButtonB.pressed([](){
        inputTracker.push_back(B);
        printInput();
    });
    con.ButtonLeft.pressed([](){
        inputTracker.push_back(LEFT);
        printInput();
    });
    con.ButtonRight.pressed([](){
        inputTracker.push_back(RIGHT);
        printInput();
    });
    con.ButtonUp.pressed([](){
        inputTracker.push_back(UP);
        printInput();
    });
    con.ButtonDown.pressed([](){
        inputTracker.push_back(DOWN);
        printInput();
    });
    con.ButtonL1.pressed([](){
        inputTracker.push_back(L1);
        printInput();
    });
    con.ButtonL2.pressed([](){
        inputTracker.push_back(L2);
        printInput();
    });
    con.ButtonR1.pressed([](){
        inputTracker.push_back(R1);
        printInput();
    });
    con.ButtonR2.pressed([](){
        inputTracker.push_back(R2);
        printInput();
    });
}

void konamiCode(){
    if(inputTracker.size() >= 9){
        if((inputTracker.at(0) == UP) && (inputTracker.at(1) == DOWN)  && (inputTracker.at(2) == UP) && (inputTracker.at(3) == DOWN)
        && (inputTracker.at(4) == LEFT) && (inputTracker.at(5) == RIGHT) && (inputTracker.at(6) == B) && (inputTracker.at(7) == A)
        && (inputTracker.at(8) == R2)){
            if(runDrive){
                printf("Turning off drive\n");
                runDrive = false;
            }
            else if(!runDrive){
                printf("Turning on drive\n");
                runDrive = true;
            }
            inputTracker.clear();
        }
    }
    if(inputTracker.size() > 9){
        inputTracker.erase(inputTracker.begin());
    }

}

void testing() {

    class DebugCommand : public AutoCommand {
      public:
        bool run() override {
            drive_sys.stop();
            Pose2d pos = odom.get_position();
            // printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
            // printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
            while (true) {
                // double left = (double)con.Axis3.position() / 100;
                // double right = (double)con.Axis2.position() / 100;

                // drive_sys.drive_tank(left, right, 1, TankDrive::BrakeType::None);

                vexDelay(100);
            }
            return true;
        }
    };

    con.ButtonX.pressed([]() {
        if(!runDrive){
            printf("running test");
            CommandController cc{
            new Async(new FunctionCommand([]() {
                while (true) {
                    printf(
                        "ODO X: %f ODO Y: %f, ODO ROT: %f TURNPID ERROR: %f\n", odom.get_position().x(),
                        odom.get_position().y(), odom.get_position().rotation().degrees(), turn_pid.get_error()
                    );
                    vexDelay(100);
                }
                return true;
            })),
            drive_sys.TurnDegreesCmd(90, 1),
            // drive_sys.TurnDegreesCmd(30, 1)->withTimeout(3),
            // drive_sys.TurnDegreesCmd(45, 1)->withTimeout(3),
            // drive_sys.TurnDegreesCmd(90, 1)->withTimeout(3),
            // drive_sys.TurnDegreesCmd(180, 1)->withTimeout(3),
            };
            cc.run();
        }
    });
        
}

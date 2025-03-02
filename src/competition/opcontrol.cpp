#include "competition/opcontrol.h"
#include "competition/autonomous.h"
#include "robot-config.h"
#include "vex.h"

void testing();

void auto__();

/**
 * Main entrypoint for the driver control period
 */

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

    // ================ INIT ================

    while (true) {
        if (!conveyor_button.pressing() && !conveyor_button_rev.pressing()) {
            intake_sys.intake_stop();
            intake_sys.conveyor_stop();
        }
        // if(turnBool){
        //     OdometryBase *odombase = &odom;
        //     pose_t pos = odombase->get_position();
        //     printf("ODO X: %.2f, Y: %.2f, R:%.2f, PID Error: %f\n", pos.x, pos.y, pos.rot, turn_pid.get_error());
        // }
        double left = (double)con.Axis3.position() / 100;
        double right = (double)con.Axis2.position() / 100;

        // drive_sys.drive_tank(left, right, 1, TankDrive::BrakeType::None);

        vexDelay(20);
    }

    // ================ PERIODIC ================
}

// vex::task printOdomTask = vex::task(printOdom);

void testing() {

    con.ButtonUp.pressed([](){
        turnTuner.startTuning();
        printf("%f\n",turnTuner.getTurnBool());
    });

    con.ButtonX.pressed([]() {
        turnTuner.stopTuning();
        printf("%f\n",turnTuner.getTurnBool());
    });
}
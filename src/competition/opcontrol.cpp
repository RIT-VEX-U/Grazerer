#include "competition/opcontrol.h"
#include "robot-config.h"
#include "vex.h"

const vex::controller::button &goal_grabber = con.ButtonB;
const vex::controller::button &conveyor_button = con.ButtonR1;
const vex::controller::button &conveyor_button_rev = con.ButtonR2;

const vex::controller::button &wallstake_toggler = con.ButtonL1;
const vex::controller::button &wallstake_stow = con.ButtonL2;
const vex::controller::button &wallstake_alliancestake = con.ButtonDown;

void testing();

void auto__();

int goal_counter = 0;

/**
 * Main entrypoint for the driver control period
 */
void opcontrol() {
    // testing();

    goal_grabber.pressed([]() {
        goal_grabber_sol.set(!goal_grabber_sol);
        goal_counter = 50;
    });

    conveyor_button.pressed([]() {
        conveyor.spin(vex::directionType::fwd, 10, vex::volt);
        intake();
        // mcglight_board.set(true);
    });
    conveyor_button_rev.pressed([]() {
        conveyor.spin(vex::directionType::rev, 10, vex::volt);
        outtake();
    });

    wallstake_toggler.pressed([]() {
        wallstake_mech.hold = true;
        if (wallstake_mech.get_angle().degrees() > 180) {
            wallstake_mech.set_setpoint(from_degrees(173));
        } else if (wallstake_mech.get_angle().degrees() < 180) {
            wallstake_mech.set_setpoint(from_degrees(50));
        }
    });

    wallstake_stow.pressed([]() {
        wallstake_mech.hold = true;
        wallstake_mech.set_setpoint(from_degrees(198.5));
    });

    wallstake_alliancestake.pressed([]() {
        wallstake_mech.hold = true;
        wallstake_mech.set_setpoint(from_degrees(0));
    });

    // ================ INIT ================

    while (true) {
        if (!conveyor_button.pressing() && !conveyor_button_rev.pressing()) {
            conveyor.stop();
            intake(0);
            // mcglight_board.set(false);
        }

        double left = (double)con.Axis3.position() / 100;
        double right = (double)con.Axis2.position() / 100;

        drive_sys.drive_tank(left, right, 1, TankDrive::BrakeType::None);

        pose_t pos = odom.get_position();
        // printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);

        if (goal_sensor.objectDistance(vex::mm) < 25 && goal_counter == 0) {
            goal_grabber_sol.set(true);
        }

        if (goal_counter > 0) {
            goal_counter--;
        }

        vexDelay(20);
    }

    // ================ PERIODIC ================
}

void testing() {

    class DebugCommand : public AutoCommand {
      public:
        bool run() override {
            drive_sys.stop();
            pose_t pos = odom.get_position();
            printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
            while (true) {
                double left = (double)con.Axis3.position() / 100;
                double right = (double)con.Axis2.position() / 100;

                drive_sys.drive_tank(left, right, 1, TankDrive::BrakeType::None);

                vexDelay(100);
            }
            return true;
        }
    };

    con.ButtonA.pressed([]() {
        CommandController cc{
          new Async(new FunctionCommand([]() {
              while (true) {
                  OdometryBase *odombase = &odom;
                  pose_t pos = odombase->get_position();
                  printf("ODO X: %.2f, Y: %.2f, R:%.2f, Concurr: %f\n", pos.x, pos.y, pos.rot, conveyor.current());
                  vexDelay(100);

                  if ((conveyor.current() > 2) && conveyor.velocity(rpm) < 0.5) {
                      printf("Conveyor Stalling");
                      conveyor_intake(-12);
                      vexDelay(500);
                      conveyor_intake(12);
                  }
              }
              return true;
          })),
        };
        cc.run();
    });
}

void auto__() {
    CommandController cc{
      // odom.SetPositionCmd({.x = 9.5, .y = 72, .rot = 0}),

      new Async(new FunctionCommand([]() {
          while (true) {
              OdometryBase *odombase = &odom;
              pose_t pos = odombase->get_position();
              printf("ODO X: %.2f, Y: %.2f, R:%.2f, Concurr: %f\n", pos.x, pos.y, pos.rot, conveyor.current());
              vexDelay(100);

              if ((conveyor.current() > 2) && conveyor.velocity(rpm) < 0.5) {
                  printf("Conveyor Stalling");
                  conveyor_intake(-12);
                  vexDelay(500);
                  conveyor_intake(12);
              }

              if (goal_sensor.objectDistance(vex::mm) < 25 && goal_counter == 0) {
                  goal_grabber_sol.set(true);
              }

              if (goal_counter > 0) {
                  goal_counter--;
              }
          }
          return true;
      })),

      // First Ring

      // drive_sys.DriveForwardCmd(24, fwd, 0.6)->withTimeout(2),
      // drive_sys.TurnToHeadingCmd(90, 0.6),
      // drive_sys.DriveForwardCmd(24, fwd, 0.6)->withTimeout(2),
      // drive_sys.TurnToHeadingCmd(180, 0.6),
      // drive_sys.DriveForwardCmd(24, fwd, 0.6)->withTimeout(2),
      // drive_sys.TurnToHeadingCmd(270, 0.6),
      // drive_sys.DriveForwardCmd(24, fwd, 0.6)->withTimeout(2),
      // drive_sys.TurnToHeadingCmd(360, 0.6),

      // intake_command(),
      drive_sys.DriveToPointCmd({29.5, 80}, vex::reverse, 0.5)->withTimeout(4),

    };
    cc.run();
}
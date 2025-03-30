#include "competition/opcontrol.h"
#include "competition/autonomous.h"
#include "robot-config.h"
#include "vex.h"

/**
 * Main entrypoint for the driver control period
 */
void opcontrol() {

    // ================ INIT ================

    while (true) {
        OdometryBase *odombase = &odom;
        Pose2d pos = odombase->get_position();

        double f = con.Axis3.position() / 200.0;
        double s = con.Axis1.position() / 200.0;
            
        drive_sys.drive_arcade(f, s, 1, TankDrive::BrakeType::None);

        vexDelay(20);
    }

    // ================ PERIODIC ================
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
        //   drive_sys.TurnDegreesCmd(15, 1),
          // drive_sys.TurnDegreesCmd(30, 1)->withTimeout(3),
          // drive_sys.TurnDegreesCmd(45, 1)->withTimeout(3),
          // drive_sys.TurnDegreesCmd(90, 1)->withTimeout(3),
          // drive_sys.TurnDegreesCmd(180, 1)->withTimeout(3),
        };
        cc.run();
    });
}
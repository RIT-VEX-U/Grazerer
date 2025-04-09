#include "competition/opcontrol.h"
#include "competition/autonomous.h"
#include "robot-config.h"
#include "vex.h"
#include "mazegame/mazegame.h"
#include "mazegame/maze_path.h"

/**
 * Main entrypoint for the driver control period
 */
void opcontrol() {
    // Pose2d maze_game{108, 84, from_degrees(0)};
    // odom.set_position(maze_game);

    MazeGame::init_boundary_lines();
    bool is_end = false;
    timer game_timer;

    // maze_auto();

    // ================ INIT ================

    while (true) {
        OdometryBase *odombase = &odom;
        Pose2d pos = odombase->get_position();
        // Pose2d pos = odom.get_position();

        double f = con.Axis3.position() / 200.0;
        double s = con.Axis1.position() / 200.0;
            
        drive_sys.drive_arcade(f, s, 1, TankDrive::BrakeType::None);

        MazeGame::is_single_penalty();   
        MazeGame::is_super_mega_ultra_penalty();

        // printf("Roll: %f ", imu.roll());
        printf(
            "ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x(), pos.y(), pos.rotation().degrees()
        );

        con.Screen.clearScreen();
        con.Screen.setCursor(1, 0);
        con.Screen.print("Penalties: %d", MazeGame::num_penalties + MazeGame::num_smups);

        if(pos.x() > 48 && pos.x() < 96 && pos.y() > 72)
        {
            int score = game_timer.time(sec) + (MazeGame::num_penalties * 5) + (MazeGame::num_smups * 45);
            con.Screen.clearScreen();
            con.Screen.setCursor(1, 0);
            con.Screen.print("Penalties: %d", MazeGame::num_penalties + MazeGame::num_smups);
            con.Screen.setCursor(2, 0);
            con.Screen.print("Final Time: %d", score);
            drive_sys.stop();
            return;
        }
        vexDelay(10);
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
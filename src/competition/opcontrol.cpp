#include "competition/opcontrol.h"
#include "competition/autonomous.h"
#include "robot-config.h"
#include "vex.h"

void testing();

void auto__();

#include "../core/include/devices/cobs_serial_device.h"
class DumbDevice : public COBSSerialDevice {
  public:
    typedef struct {
        float x;
        float y;
        float h;
    } otos_pose2d_t;

    DumbDevice(int32_t port, int32_t baud) : COBSSerialDevice(port, baud) {}
    void cobs_packet_callback(const COBSSerialDevice::Packet &pac) {
        otos_pose2d_t pose = {0};
        float velf = 0;
        float avelf = 0;
        float accf = 0;
        float aaccf = 0;
        float *floats = (float *)pac.data();
        pose.x = floats[0];
        pose.y = floats[1];
        pose.h = floats[2];

        printf("Serial (%.2f, %.2f), %.2f\n", pose.x, pose.y, pose.h);
    }
};

DumbDevice dev{vex::PORT1, 115200};
/**
 * Main entrypoint for the driver control period
 */
void opcontrol() {
    bool calc_vel_acc_on_brain = false;
    pose_t initial_pose = {18.5, 96, 0};
    pose_t sensor_offset = pose_t{-3.83, 0.2647, 270};
    //
    uint8_t raw[(sizeof(float)) * 6 + sizeof(calc_vel_acc_on_brain)];
    uint8_t cobs_encoded[sizeof(raw) + 1];
    //
    float initialx = (float)initial_pose.x;
    float initialy = (float)initial_pose.y;
    float initialrot = (float)initial_pose.rot;
    //
    float offsetx = (float)sensor_offset.x;
    float offsety = (float)sensor_offset.y;
    float offsetrot = (float)sensor_offset.rot;

    memcpy(&raw[0], &initialx, sizeof(float));
    memcpy(&raw[4], &initialy, sizeof(float));
    memcpy(&raw[8], &initialrot, sizeof(float));
    memcpy(&raw[12], &offsetx, sizeof(float));
    memcpy(&raw[16], &offsety, sizeof(float));
    memcpy(&raw[20], &offsetrot, sizeof(float));
    memcpy(&raw[24], &calc_vel_acc_on_brain, sizeof(bool));
    //
    COBSSerialDevice::Packet pac{raw, raw + 25};
    for (int i = 0; i < 25; i++) {
        printf("%02x ", raw[i]);
    }
    printf("\n");
    vexDelay(1500);
    dev.send_cobs_packet(pac);
    while (true) {
        double left = (double)con.Axis3.position() / 100;
        double right = (double)con.Axis2.position() / 100;

        drive_sys.drive_tank(left, right, 1, TankDrive::BrakeType::None);

        vexDelay(20);
    }

    vexDelay(100000);
    // autonomous();
    // return;
    // testing();
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
        OdometryBase *odombase = &odom;
        pose_t pos = odombase->get_position();
        printf("ODO X: %.2f, Y: %.2f, R:%.2f, Concurr: %f\n", pos.x, pos.y, pos.rot, conveyor.current());

        double left = (double)con.Axis3.position() / 100;
        double right = (double)con.Axis2.position() / 100;

        drive_sys.drive_tank(left, right, 1, TankDrive::BrakeType::None);

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
                    "ODO X: %f ODO Y: %f, ODO ROT: %f TURNPID ERROR: %f\n", odom.get_position().x,
                    odom.get_position().y, odom.get_position().rot, turn_pid.get_error()
                  );
                  vexDelay(100);
              }
              return true;
          })),
          drive_sys.TurnDegreesCmd(15, 1),
          // drive_sys.TurnDegreesCmd(30, 1)->withTimeout(3),
          // drive_sys.TurnDegreesCmd(45, 1)->withTimeout(3),
          // drive_sys.TurnDegreesCmd(90, 1)->withTimeout(3),
          // drive_sys.TurnDegreesCmd(180, 1)->withTimeout(3),
        };
        cc.run();
    });
}

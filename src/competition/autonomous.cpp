#include "competition/autonomous.h"
#include "robot-config.h"
/**
 * Main entrypoint for the autonomous period
 */

void skills();

void autonomous() {
    intake_sys.color_sort_on();
    if (matchpath == MatchPaths::BLUE_SAFE_AUTO) {
        auto_blue_safe();
    } else if (matchpath == MatchPaths::RED_SAFE_AUTO) {
        auto_red_safe();
    } else if (matchpath == MatchPaths::BASIC_SKILLS) {
        skills_basic();
    }
}

class DebugCommand : public AutoCommand {
  public:
    bool run() override {
        drive_sys.stop();
        Pose2d pos = odom.get_position();
        printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x(), pos.y(), pos.rotation().degrees());
        while (true) {
            double f = con.Axis3.position() / 200.0;
            double s = con.Axis1.position() / 200.0;
            // double left_enc_start_pos = left_enc.position(vex::rotationUnits::rev);
            drive_sys.drive_arcade(f, s, 1, TankDrive::BrakeType::None);
            Pose2d pos = odom.get_position();
            printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x(), pos.y(), pos.rotation().degrees());
            // printf("ENC LEFT REV: %.2f, ENC RIGHT POS: %.2f, ENC BACK POS: %.2f\n",
            // left_enc.position(vex::rotationUnits::deg), right_enc.position(vex::rotationUnits::deg),
            // front_enc.position(vex::rotationUnits::deg)); if (left_enc.position(vex::rotationUnits::rev) >= 1.0) {
            //     break;
            // }
            vexDelay(100);
        }
        return false;
    }
};

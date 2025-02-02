#pragma once
#include "vex.h"
#include "core.h"
#include "../core/include/subsystems/odometry/odometry_serial.h"
#include "wallstake_mech.h"
#include "auto-red-safe.cpp"

#define WALLSTAKE_POT_OFFSET 

extern vex::brain brain;
extern vex::controller con;


// ================ INPUTS ================
// Digital sensors

// Analog sensors
extern CustomEncoder Left_enc;
extern CustomEncoder right_enc;
extern CustomEncoder front_enc;

// ================ OUTPUTS ================
// Motors
extern vex::motor left_back_bottom;
extern vex::motor left_center_bottom;
extern vex::motor left_front_top;
extern vex::motor left_back_top;

extern vex::motor right_back_bottom;
extern vex::motor right_center_bottom;
extern vex::motor right_front_top;
extern vex::motor right_back_top;

extern vex::motor conveyor;
extern vex::motor intake_motor;

extern vex::motor wallstake_left;
extern vex::motor wallstake_right;
extern vex::motor_group wallstake_motors;

extern Rotation2d initial;
extern Rotation2d tolerance;
extern double pot_offset;
extern vex::pot wall_pot;
extern WallStakeMech wallstake_mech;
extern vex::optical color_sensor;
extern vex::digital_out mcglight_board;

void intake(double volts);

void intake();

void outtake(double volts);

void outtake();

void conveyor_intake(double volts);

void conveyor_intake();

void conveyor_outtake(double volts);

void conveyor_outtake();


void intake_spin(double volts);

extern vex::motor_group left_drive_motors;
extern vex::motor_group right_drive_motors;

// Pneumatics
extern vex::digital_out goal_grabber_sol;
extern vex::inertial imu;

extern vex::distance goal_sensor;

extern vex::pot wall_pot;



// ================ SUBSYSTEMS ================
extern PID drive_pid;
extern PID turn_pid;
extern PID turn_pidBigI;
extern MotionController::m_profile_cfg_t drive_motioncontroller_cfg;
extern MotionController drive_motioncontroller;

extern PID::pid_config_t correction_pid_cfg;
extern OdometrySerial odom;
extern OdometryTank tankodom;

extern robot_specs_t robot_cfg;
extern TankDrive drive_sys;

// ================ UTILS ================
enum MatchPaths{
	BLUE_SAFE_AUTO,
	RED_SAFE_AUTO,
	BASIC_SKILLS,
};

extern bool color_sort_on;
extern bool conveyor_started;
extern int color_sensor_counter;
extern MatchPaths matchpath;
extern bool blue_alliance();
void robot_init();

AutoCommand *intake_command(double amt = 10.0) {
	return new FunctionCommand([=]() {
		intake(amt);
		return true;
	});
}

AutoCommand *outtake_command(double amt = 10.0) {
	return new FunctionCommand([=]() {
		intake(-amt);
		return true;
	});
}


AutoCommand *stop_intake() {
	return new FunctionCommand([=]() {
		intake(0);
		return true;
	});
}

AutoCommand *conveyor_intake_command(double amt = 10.0) {
	return new FunctionCommand([=]() {
		conveyor_intake(amt);
		conveyor_started = true;
		return true;
	});
}

AutoCommand *conveyor_stop_command() {
	return new FunctionCommand([=]() {
		conveyor_intake(0);
		conveyor_started = false;
		return true;
	});
}

AutoCommand *goal_grabber_command(bool value) {
	return new FunctionCommand([=]() {
		goal_grabber_sol.set(value);
		return true;
	});
}

AutoCommand *alliance_score_command(bool hold = true) {
	return new FunctionCommand([=]() {
		wallstake_mech.hold = hold;
        wallstake_mech.set_setpoint(from_degrees(0));
		return true;
	});
}

AutoCommand *stow_command() {
	return new FunctionCommand([=]() {
		wallstake_mech.hold = true;
        wallstake_mech.set_setpoint(from_degrees(198.5));
		return true;
	});
}

AutoCommand *handoff_command() {
	return new FunctionCommand([=]() {
		wallstake_mech.hold = true;
        wallstake_mech.set_state(HANDOFF);
		return true;
	});
}

AutoCommand *wallstake_command() {
	return new FunctionCommand([=]() {
		wallstake_mech.hold = true;
        wallstake_mech.set_setpoint(from_degrees(80));
		return true;
	});
}
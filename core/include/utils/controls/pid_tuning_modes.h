#pragma Once
#include "../core/include/subsystems/tank_drive.h"
#include "../core/include/utils/controls/pid.h"
#include "../core/include/subsystems/odometry/odometry_base.h"
#include "../core/include/utils/controls/feedback_base.h"
class PIDTuner{
    public:
    enum PIDType{
        DRIVEPID,
        TURNPID,
    };
    struct pid_tuner_cfg{
        TankDrive drivesys;
        PIDType pid_type;
    };
    PIDTuner(pid_tuner_cfg in_tuner_cfg);
    void print_pid_data();
    void inputPID();
    static int tuner_fn(void *ptr);
    pid_tuner_cfg tuner_cfg;
    bool getPrintBool(){
        return printBool;
    }
    bool setprintBool(bool newPrintBool){
        printBool = newPrintBool;
    }

    void stopTuning(){
        turnBool = false;
        printBool = false;
    }
    void startTuning(){
        turnBool = false;
        printBool = false;
        inputPID();
        turnBool = true;
        printBool = true;
    }

    bool getTurnBool(){
        return turnBool;
    }
    bool setTurnBool(bool newTurnBool){
        turnBool = newTurnBool;
    }
    private:
    bool printBool = false;
    bool turnBool = false;
    PIDType type;
    TankDrive drive_sys = tuner_cfg.drivesys;
    PID::pid_config_t pid_cfg;
    PID *pid;
    
    vex::task task;
    OdometryBase *odom;
    pose_t pos;
    
    double setpoint;
};
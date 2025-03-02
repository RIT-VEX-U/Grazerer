#pragma Once
#include "../core/include/utils/controls/pid_tuning_modes.h"
#include <iostream>

PIDTuner::PIDTuner(pid_tuner_cfg in_tuner_cfg) : tuner_cfg(in_tuner_cfg){ 
    if(type == PIDType::DRIVEPID){
        pid_cfg = drive_sys.config->drive_feedback->pid_config;
        pid = drive_sys.config->drive_feedback;
    }
    else if(type == PIDType::TURNPID){
        pid_cfg = drive_sys.config->turn_feedback->pid_config;
        pid = drive_sys.config->turn_feedback;
    }
    task = vex::task(tuner_fn, this); 
}

void PIDTuner::print_pid_data(){
    pos = odom->get_position();
    printf("\nODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
    printf("PID ERROR: %.4f, PID OUT: %.4f\n", pid->get_error(), pid->get());
        printf("P: %f, I: %f, D:%f, Setpoint: %.2f\n", pid_cfg.p, pid_cfg.i, pid_cfg.d, setpoint);
}

void PIDTuner::inputPID(){
    printf("\nP: ");
    std::cin >> pid_cfg.p;
    printf("\nI: ");
    std::cin >> pid_cfg.i;
    printf("\nD: ");
    std::cin >> pid_cfg.d;
    printf("\nSetpoint: ");
    std::cin >> setpoint;
}

int PIDTuner::tuner_fn(void *ptr){
    PIDTuner &self = *(PIDTuner *)ptr;
    while(true){
        if(self.turnBool == true){
            self.drive_sys.turn_to_heading(self.setpoint, 1);
        }
        if(self.printBool == true){
            self.print_pid_data();
        }

        vexDelay(20);
    }
    return 0;
}
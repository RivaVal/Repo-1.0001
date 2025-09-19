


//===========================================================================
//	📄 motor_controller_private.h
//===========================================================================
cpp

#pragma once

#ifndef MOTOR_CONTROLLER_PRIVATE_H
#define MOTOR_CONTROLLER_PRIVATE_H

#include "motor_config.h"
#include <driver/mcpwm_prelude.h>

struct MotorResources {
    mcpwm_timer_handle_t timer = nullptr;
    mcpwm_oper_handle_t oper = nullptr;
    mcpwm_cmpr_handle_t comparator = nullptr;
    mcpwm_gen_handle_t generator = nullptr;
    int current_speed = 0;
    int pin = -1;
    
    bool is_initialized() const {
        return timer && oper && comparator && generator;
    }
};

#endif


#include "modeMPC.h"
#include "Sub.h"
#include "AP_Motors/AP_Motors.h"
#include "AC_AttitudeControl/AC_AttitudeControl_Sub.h"

// Initialize the MPC control mode
bool ModeMPC::init(bool ignore_checks)
{
    // 执行 MPC 模式特定的初始化
    sub.set_neutral_controls();  // 确保设置中性控制

    // 初始化 MPC 控制器
    sub.mpc_controller.init();  // 假设有 mpc_controller 对象

    return true;
}

// Run the MPC controller
void ModeMPC::run()
{
    // 如果未解锁，则保持所有内容处于空闲状态
    if (!sub.motors.armed()) {
        sub.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        sub.attitude_control.set_throttle_out(0, true, sub.g.throttle_filt);
        sub.attitude_control.relax_attitude_controllers();
        return;
    }

    // 当已解锁时，将电机状态设置为油门无限制
    sub.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 运行 MPC 控制算法以确定期望的电机输出
    float roll_output, pitch_output, yaw_output, throttle_output;
    sub.mpc_controller.calculate_control(roll_output, pitch_output, yaw_output, throttle_output);

    // 设置电机输出
    sub.motors.set_roll(roll_output);
    sub.motors.set_pitch(pitch_output);
    sub.motors.set_yaw(yaw_output);
    sub.motors.set_throttle(throttle_output);
}

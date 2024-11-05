#pragma once

#include "mode.h"

class ModeMPC : public Mode {
public:
    // 使用基类的构造函数
    using Mode::Mode;

    // 定义 MPC 模式的相关方法
    void run() override;
    bool init(bool ignore_checks) override;

    // 需要覆盖的纯虚函数
    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }  // 如果 MPC 是自动驾驶模式，则返回 true，否则返回 false

    // 获取模式名称
    const char *name() const override { return "MPC"; }
    const char *name4() const override { return "MPC "; }
};
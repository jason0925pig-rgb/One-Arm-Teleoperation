#ifndef ZX_GRIPPER_H
#define ZX_GRIPPER_H

#include <modbus/modbus.h>
#include <mutex>
#include <thread>
#include <chrono>
#include <stdexcept>
#include <string>

// Register Map (Holding, 0x03/0x06/0x10)
const int REG_ENABLE = 0x0100;

// Temporary-zone motion (write)
const int REG_TMP_POS_H = 0x0102; // high 16
const int REG_TMP_POS_L = 0x0103; // low  16
const int REG_TMP_SPEED = 0x0104; // 0~100 (% of max speed)
const int REG_TMP_FORCE = 0x0105; // 0~100 (% of max torque)
const int REG_TMP_ACCEL = 0x0106;
const int REG_TMP_DECEL = 0x0107;
const int REG_TMP_TRIGGER = 0x0108; // 0:idle, 1:trigger

// Status/Feedback registers
const int REG_TORQUE_REACHED = 0x0601;
const int REG_POS_REACHED = 0x0602;
const int REG_READY = 0x0604;
const int REG_POS_FB_H = 0x0609;
const int REG_POS_FB_L = 0x060A;
const int REG_ALARM = 0x0612;

class ZX_gripper
{
private:
    modbus_t *ctx;
    std::mutex mtx;
    int slave_id;

public:
    ZX_gripper(const char *port, int slave_id = 1, int baudrate = 115200, float timeout = 0.3);
    ~ZX_gripper();

    // Low-level helpers
    void write_register(int addr, int value);
    void write_registers(int addr, const uint16_t *values, int count);
    uint16_t read_register(int addr);

    // Enable/Disable
    void enable(bool enable = true);

    // Temporary-Zone Motion
    void set_temp_position_mm(int position_mm);
    void set_temp_speed_pct(int speed_pct);
    void set_temp_force_pct(int force_pct);
    void set_temp_accel(int accel);
    void set_temp_decel(int decel);
    void trigger_temp_move();
    void temp_move(int position_mm, int speed_pct = 100, int force_pct = 60,
                   int accel = 2000, int decel = 2000, bool trigger = true);

    // Status/Feedback
    bool torque_reached();
    bool position_reached();
    bool ready();
    uint32_t feedback_position();
    uint16_t read_alarm();

    // Utilities
    bool wait_until_ready(double timeout = 5.0, double poll = 0.02);
    std::string wait_until_pos_or_torque(double timeout = 5.0, double poll = 0.02);
};

#endif // ZX_GRIPPER_H
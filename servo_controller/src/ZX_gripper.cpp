// #include "../include/ZX_gripper.h"
#include "ZX_gripper.h"
#include <cmath>
#include <iostream>

ZX_gripper::ZX_gripper(const char *port, int slave_id, int baudrate, float timeout)
    : slave_id(slave_id)
{
    // Initialize Modbus context
    ctx = modbus_new_rtu(port, baudrate, 'N', 8, 1);
    if (ctx == nullptr)
    {
        throw std::runtime_error("Failed to create Modbus context");
    }

    // libmodbus expects (whole seconds, remaining microseconds). The supplied
    // code had these fields reversed, turning 0.3 s into an extremely long
    // timeout when a device did not reply.
    if (!std::isfinite(timeout) || timeout <= 0.0F)
    {
        modbus_free(ctx);
        throw std::invalid_argument("timeout must be a positive finite value");
    }
    const auto timeout_seconds = static_cast<uint32_t>(timeout);
    const auto timeout_microseconds = static_cast<uint32_t>(
        (timeout - static_cast<float>(timeout_seconds)) * 1000000.0F);
    if (modbus_set_response_timeout(
            ctx, timeout_seconds, timeout_microseconds) == -1)
    {
        modbus_free(ctx);
        throw std::runtime_error("Failed to set Modbus response timeout");
    }

    // Set slave ID
    modbus_set_slave(ctx, slave_id);

    // Connect
    if (modbus_connect(ctx) == -1)
    {
        modbus_free(ctx);
        throw std::runtime_error("Modbus connection failed");
    }
}

ZX_gripper::~ZX_gripper()
{
    if (ctx)
    {
        modbus_close(ctx);
        modbus_free(ctx);
    }
}

void ZX_gripper::write_register(int addr, int value)
{
    std::lock_guard<std::mutex> lock(mtx);
    if (modbus_write_register(ctx, addr, value) == -1)
    {
        throw std::runtime_error("Modbus write register failed");
    }
}

void ZX_gripper::write_registers(int addr, const uint16_t *values, int count)
{
    std::lock_guard<std::mutex> lock(mtx);
    if (modbus_write_registers(ctx, addr, count, values) == -1)
    {
        throw std::runtime_error("Modbus write registers failed");
    }
}

uint16_t ZX_gripper::read_register(int addr)
{
    std::lock_guard<std::mutex> lock(mtx);
    uint16_t value;
    if (modbus_read_registers(ctx, addr, 1, &value) == -1)
    {
        throw std::runtime_error("Modbus read register failed");
    }
    return value;
}

void ZX_gripper::enable(bool enable)
{
    write_register(REG_ENABLE, enable ? 1 : 0);
}

void ZX_gripper::set_temp_position_mm(int position_mm)
{
    if (position_mm < 0)
    {
        throw std::invalid_argument("position_mm must be within 0..0xFFFFFFFF");
    }
    uint16_t hi = (position_mm >> 16) & 0xFFFF;
    uint16_t lo = position_mm & 0xFFFF;
    uint16_t values[2] = {hi, lo};
    write_registers(REG_TMP_POS_H, values, 2);
}

void ZX_gripper::set_temp_speed_pct(int speed_pct)
{
    if (speed_pct < 0 || speed_pct > 100)
    {
        throw std::invalid_argument("speed_pct must be 0..100");
    }
    write_register(REG_TMP_SPEED, speed_pct);
}

void ZX_gripper::set_temp_force_pct(int force_pct)
{
    if (force_pct < 0 || force_pct > 100)
    {
        throw std::invalid_argument("force_pct must be 0..100");
    }
    write_register(REG_TMP_FORCE, force_pct);
}

void ZX_gripper::set_temp_accel(int accel)
{
    write_register(REG_TMP_ACCEL, accel);
}

void ZX_gripper::set_temp_decel(int decel)
{
    write_register(REG_TMP_DECEL, decel);
}

void ZX_gripper::trigger_temp_move()
{
    write_register(REG_TMP_TRIGGER, 1);
}

void ZX_gripper::temp_move(int position_mm, int speed_pct, int force_pct,
                           int accel, int decel, bool trigger)
{
    set_temp_position_mm(position_mm);
    set_temp_speed_pct(speed_pct);
    set_temp_force_pct(force_pct);
    set_temp_accel(accel);
    set_temp_decel(decel);
    if (trigger)
    {
        trigger_temp_move();
    }
}

bool ZX_gripper::torque_reached()
{
    return read_register(REG_TORQUE_REACHED) != 0;
}

bool ZX_gripper::position_reached()
{
    return read_register(REG_POS_REACHED) != 0;
}

bool ZX_gripper::ready()
{
    return read_register(REG_READY) != 0;
}

uint32_t ZX_gripper::feedback_position()
{
    uint16_t hi = read_register(REG_POS_FB_H);
    uint16_t lo = read_register(REG_POS_FB_L);
    return (static_cast<uint32_t>(hi) << 16) | lo;
}

uint16_t ZX_gripper::read_alarm()
{
    return read_register(REG_ALARM);
}

bool ZX_gripper::wait_until_ready(double timeout, double poll)
{
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::duration<double>(timeout);

    while (std::chrono::steady_clock::now() < deadline)
    {
        try
        {
            if (ready())
            {
                return true;
            }
        }
        catch (const std::exception &)
        {
            // Ignore and retry
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(poll));
    }
    return false;
}

std::string ZX_gripper::wait_until_pos_or_torque(double timeout, double poll)
{
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::duration<double>(timeout);

    while (std::chrono::steady_clock::now() < deadline)
    {
        try
        {
            if (position_reached())
            {
                return "position";
            }
            if (torque_reached())
            {
                return "torque";
            }
        }
        catch (const std::exception &)
        {
            // Ignore and retry
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(poll));
    }
    return "timeout";
}

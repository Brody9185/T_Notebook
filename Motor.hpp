#pragma once
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <atomic>
#include "libs/include/T_Lib/util.hpp"
#include "util.hpp"

using namespace T_Lib::util;

namespace T_Lib {
class T_Motor {
private:
    pros::Motor motor;
    pros::Task* control_task;

    std::atomic<double> target_rpm;
    std::atomic<bool> running;

    double kV_low = 9;
    double kP_low = 12.5;
    double kV_high = 9;
    double kP_high = 11.5;

    std::atomic<bool> load_comp_enabled;
    std::atomic<double> load_kBoost;
    std::atomic<int> load_threshold;

    std::atomic<bool> min_torque_enabled;
    std::atomic<int> min_voltage_user;
    std::atomic<int> min_voltage;
    int min_voltage_low;

    double base_rpm;

    std::atomic<bool> reversed{false};  // reversed flag

    // --- Slew limiter ---
    std::atomic<bool> slew_enabled{false};      
    std::atomic<double> max_slew_change{500.0}; // default 500

    // --- PID Enable toggle ---
    std::atomic<bool> pid_enabled{true};  // default: PID active

    static void controlLoopTask(void* param) {
        T_Motor* self = static_cast<T_Motor*>(param);
        double last_voltage = 0.0;

        while (self->running) {
            // If PID is disabled, skip all control and just wait
            if (!self->pid_enabled.load()) {
                pros::delay(10);
                continue;
            }

            double rpm = self->motor.get_actual_velocity();
            double target = self->target_rpm;
            double error = target - rpm;

            if (std::abs(error) < 2.0) {
                error = 0.0;
            }

            double kV_used = (std::abs(target) <= self->base_rpm) ? self->kV_low : self->kV_high;
            double kP_used = (std::abs(target) <= self->base_rpm) ? self->kP_low : self->kP_high;
            double voltage = kV_used * target + kP_used * error;

            if (self->load_comp_enabled) {
                if (std::abs(error) > self->load_threshold) {
                    voltage += error * self->load_kBoost;
                }
            }

            if (std::abs(target) <= 20) {
                self->min_voltage = self->min_voltage_low;
            } else {
                self->min_voltage = self->min_voltage_user.load();
            }

            if (self->min_torque_enabled && std::abs(target) > 1) {
                int min_voltage = self->min_voltage.load();
                if (std::abs(voltage) < min_voltage) {
                    voltage = (voltage > 0) ? min_voltage : -min_voltage;
                }
            }

            // --- Apply slew limiter if enabled ---
            if (self->slew_enabled) {
                double change = voltage - last_voltage;
                double max_change = self->max_slew_change.load();
                if (std::abs(change) > max_change) {
                    voltage = last_voltage + (change > 0 ? max_change : -max_change);
                }
            }
            last_voltage = voltage;

            // Apply reversed flag inversion
            if (self->reversed.load()) {
                voltage = -voltage;
            }

            self->motor.move_voltage(static_cast<int>(voltage));
            pros::delay(10);
        }
    }

public:
    T_Motor(int port, pros::v5::MotorGears gearset = Aliases::Blue, bool reversed = false)
        : motor(port, gearset, Aliases::Counts),
          target_rpm(0), running(true),
          load_comp_enabled(true), load_kBoost(4.5), load_threshold(15),
          min_torque_enabled(true),
          min_voltage_user(1200), min_voltage(1200),
          min_voltage_low(600),
          reversed(reversed)
    {
        initBaseRPM(gearset);
        control_task = new pros::Task(controlLoopTask, this, "T_MotorControl");
    }

private:
    void initBaseRPM(pros::v5::MotorGears gearset) {
        switch (gearset) {
            case pros::v5::MotorGears::red: base_rpm = 100.0; break;
            case pros::v5::MotorGears::green: base_rpm = 200.0; break;
            case pros::v5::MotorGears::blue: base_rpm = 600.0; break;
            default: base_rpm = 600.0; break;
        }

        kV_low = 10650.0 / base_rpm;
        kV_high = 12100.0 / (base_rpm * 1.1333333333333333);
    }

public:
    ~T_Motor() {
        running = false;
        pros::delay(20);
        delete control_task;
    }

    T_Motor(const T_Motor& other)
        : motor(other.motor.get_port(), other.motor.get_gearing(), Aliases::Counts),
          target_rpm(other.target_rpm.load()), running(true),
          kV_low(other.kV_low), kP_low(other.kP_low),
          kV_high(other.kV_high), kP_high(other.kP_high),
          load_comp_enabled(other.load_comp_enabled.load()),
          load_kBoost(other.load_kBoost.load()),
          load_threshold(other.load_threshold.load()),
          min_torque_enabled(other.min_torque_enabled.load()),
          min_voltage_user(other.min_voltage_user.load()),
          min_voltage(other.min_voltage.load()),
          min_voltage_low(other.min_voltage_low),
          base_rpm(other.base_rpm),
          reversed(other.reversed.load()),
          slew_enabled(other.slew_enabled.load()),
          max_slew_change(other.max_slew_change.load()),
          pid_enabled(other.pid_enabled.load())
    {
        control_task = new pros::Task(controlLoopTask, this, "T_MotorControl");
    }

    T_Motor& operator=(const T_Motor& other) {
        if (this == &other) return *this;

        new (&motor) pros::Motor(other.motor.get_port(), other.motor.get_gearing(), Aliases::Counts);

        target_rpm = other.target_rpm.load();
        running = true;

        kV_low = other.kV_low;
        kP_low = other.kP_low;
        kV_high = other.kV_high;
        kP_high = other.kP_high;

        load_comp_enabled = other.load_comp_enabled.load();
        load_kBoost = other.load_kBoost.load();
        load_threshold = other.load_threshold.load();

        min_torque_enabled = other.min_torque_enabled.load();
        min_voltage_user = other.min_voltage_user.load();
        min_voltage = other.min_voltage.load();
        min_voltage_low = other.min_voltage_low;

        base_rpm = other.base_rpm;

        reversed = other.reversed.load();
        slew_enabled = other.slew_enabled.load();
        max_slew_change = other.max_slew_change.load();
        pid_enabled = other.pid_enabled.load();

        if (control_task) {
            running = false;
            pros::delay(20);
            delete control_task;
        }

        running = true;
        control_task = new pros::Task(controlLoopTask, this, "T_MotorControl");

        return *this;
    }

    // === Control ===
    void setTargetRPM(double rpm) {
        if (reversed) rpm = -rpm;
        target_rpm = rpm;
    }

    void setTargetPercent(double percent) {
        percent = std::clamp(percent, -100.0, 100.0);
        if (reversed) percent = -percent;
        double max_rpm = base_rpm * 1.1333333333333333;
        target_rpm = (percent / 100.0) * max_rpm;
    }

    void stop() { target_rpm = 0; }

    // === PID Enable Toggle ===
    void setPIDEnabled(bool enabled) { pid_enabled = enabled; }
    bool isPIDEnabled() const { return pid_enabled.load(); }

    // === Slew control ===
    void setSlewLimitEnabled(bool enabled) { slew_enabled = enabled; }
    bool isSlewLimitEnabled() const { return slew_enabled.load(); }

    void setSlewRate(double maxChange) { max_slew_change = maxChange; }
    double getSlewRate() const { return max_slew_change.load(); }

    // === Sensor Access ===
    double getRPM() const { return motor.get_actual_velocity(); }
    double getTargetRPM() const { return target_rpm; }
    double getTemperature() const { return motor.get_temperature(); }
    double getPosition() const { return motor.get_position(); }
    int getVoltage() const { return motor.get_voltage(); }

    // === Settings ===
    void resetPosition() { motor.tare_position(); }
    void setBrakeMode(pros::v5::MotorBrake mode) { motor.set_brake_mode(mode); }

    // --- Load Compensation ---
    void setLoadCompensation(bool enabled) {
        load_comp_enabled = enabled;
        if (enabled) { load_kBoost = 4.5; load_threshold = 15; }
    }
    void setLoadCompensation(bool enabled, double kBoost) {
        load_comp_enabled = enabled;
        load_kBoost = kBoost;
        if (enabled && load_threshold <= 0) load_threshold = 15;
    }
    void setLoadCompensation(bool enabled, double kBoost, int threshold) {
        load_comp_enabled = enabled;
        load_kBoost = kBoost;
        load_threshold = threshold;
    }

    // --- PID Config ---
    void setDualConstants(double kvLow, double kpLow, double kvHigh, double kpHigh) {
        kV_low = kvLow; kP_low = kpLow;
        kV_high = kvHigh; kP_high = kpHigh;
    }
    void setLowConstants(double kvLow, double kpLow) { kV_low = kvLow; kP_low = kpLow; }
    void setHighConstants(double kvHigh, double kpHigh) { kV_high = kvHigh; kP_high = kpHigh; }

    // --- Min Torque ---
    void setMinTorque(bool enabled) { min_torque_enabled = enabled; }
    void setMinTorque(bool enabled, int minMv) { min_torque_enabled = enabled; min_voltage_user = minMv; }
    void setMinTorque(bool enabled, int minMv, int minLowMv) { min_torque_enabled = enabled; min_voltage_user = minMv; min_voltage_low = minLowMv; }

    // === Reversed control ===
    void setReversed(bool rev) { reversed = rev; }
    bool isReversed() const { return reversed.load(); }

    // === Getters ===
    double getLowKV() const { return kV_low; }
    double getLowKP() const { return kP_low; }
    double getHighKV() const { return kV_high; }
    double getHighKP() const { return kP_high; }

    double getBaseRPM() const { return base_rpm; }
    int getVoltageLimit() const { return 12000; }
    pros::v5::MotorGears getGearset() const { return motor.get_gearing(); }

    bool isLoadCompEnabled() const { return load_comp_enabled; }
    double getLoadKBoost() const { return load_kBoost; }
    int getLoadThreshold() const { return load_threshold; }

    bool isMinTorqueEnabled() const { return min_torque_enabled; }
    int getMinVoltageUser() const { return min_voltage_user; }
    int getMinVoltageLow() const { return min_voltage_low; }
    int getActiveMinVoltage() const { return min_voltage; }

    bool isSpinning() const { return std::abs(target_rpm) > 1.0; }
    bool isSpinningRaw() const { return std::abs(motor.get_actual_velocity()) > 1.0; }
};
} // namespace T_Lib

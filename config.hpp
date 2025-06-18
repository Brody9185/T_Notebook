#pragma once
#include <cstdint>
#include <vector>
#include "pros/motor_group.hpp"

namespace CONFIG {
    // Drive motor ports (individual motor IDs for drivetrain)
    extern int8_t LftMtr1, LftMtr2, LftMtr3, LftMtr4;
    extern int8_t RtMtr1, RtMtr2, RtMtr3, RtMtr4;
    extern pros::MotorGearset Base_Color;

    // IMU sensor port
    extern int IMU;

    // Drivetrain physical characteristics
    extern double Wheel_Diameter;  // in inches
    extern double Mtr_RPM;         // Motor RPM (base)
    extern double Gear_Ratio;      // Gear reduction ratio
    extern double Wheel_RPM;       // Final wheel RPM (after gearing)

    // Robot dimensions (in inches)
    extern double Wheel_Base;      // Distance between front and back wheels
    extern double Track_Width;     // Distance between left and right wheels

    // Input curve settings for arcade drive
    extern double RDB, RMO, RC;    // Right joystick curve
    extern double LDB, LMO, LC;    // Left joystick curve

    // Default drive speeds
    extern const int DRIVE_SPEED;
    extern const int TURN_SPEED;
    extern const int SWING_SPEED;

    // PID constants for linear movement
    extern double lin_P, lin_I, lin_D, lin_Start_I;

    // PID constants for angular movement
    extern double ang_P, ang_I, ang_D, ang_Start_I;

    // PID constants for turn movement
    extern double trn_P, trn_I, trn_D, trn_Start_I;

    // PID constants for swing turns
    extern double swn_P, swn_I, swn_D, swn_Start_I;

    // PID constants for heading correction
    extern double hd_P, hd_I, hd_D, hd_Start_I;

    // PID constants for boomerang-style turns
    extern double bm_P, bm_I, bm_D, bm_Start_I;

    // Linear PID exit conditions
    extern double LLET, LLE, LSET, LSE;

    // Angular PID exit conditions
    extern double ALET, ALE, ASET, ASE;

    // PID chaining thresholds
    extern double LCh, ACh, SCh;

    // Slew rate limiting (acceleration control)
    extern double LSSM, LSDM;  // Linear max and min slew
    extern double ASSM, ASDM;  // Angular max and min slew

    // Odometry tracking wheel ports and specs
    extern int Vrt1Prt, Vrt2Prt, Hr1Prt, Hr2Prt;
    extern double Vrt1D, Vrt1DTC, Vrt2D, Vrt2DTC;
    extern double Hr1D, Hr1DTC, Hr2D, Hr2DTC;

    // Motor ID arrays (for constructing motor groups)
    extern std::vector<int8_t> lemLeftMotors, lemRightMotors;
    extern std::vector<int> ezLeftMotors, ezRightMotors;

    // Pre-initialized PROS motor groups
    extern pros::MotorGroup leftMotors;
    extern pros::MotorGroup rightMotors;
}

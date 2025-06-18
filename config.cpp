#include "config.hpp"
#include "libs/include/T_Lib/api.hpp"

namespace CONFIG {
    // Drive motor ports
    int8_t LftMtr1 = -11, LftMtr2 = 12, LftMtr3 = -13, LftMtr4 = 0;
    int8_t RtMtr1 = 17, RtMtr2 = -18, RtMtr3 = 19, RtMtr4 = 0;
    pros::MotorGearset Base_Color = pros::MotorGearset::blue;

    // Inertial sensor port
    int IMU = 0;

    // Wheel and gearing parameters
    double Wheel_Diameter = T_Lib::util::Wheel_Size::Omni_4in,
           Mtr_RPM = 600,
           Gear_Ratio = 0.6666667,
           Wheel_RPM = Mtr_RPM * Gear_Ratio;

    // Robot dimensions (in inches)
    double Wheel_Base = 10, Track_Width = 10;

    // Drive input curve settings
    double RDB = 3, RMO = 10, RC = 1.019;
    double LDB = 3, LMO = 10, LC = 1.019;

    // Default drive speeds
    const int DRIVE_SPEED = 110, TURN_SPEED = 90, SWING_SPEED = 110;

    // PID settings
    double lin_P = 1, lin_I = 0, lin_D = 0, lin_Start_I = 0;
    double ang_P = 1, ang_I = 0, ang_D = 0, ang_Start_I = 0;
    double trn_P = 1, trn_I = 0, trn_D = 0, trn_Start_I = 0;
    double swn_P = 1, swn_I = 0, swn_D = 0, swn_Start_I = 0;
    double hd_P  = 1, hd_I = 0, hd_D = 0, hd_Start_I = 0;
    double bm_P  = 1, bm_I = 0, bm_D = 0, bm_Start_I = 0;

    // Exit conditions (timeouts and thresholds)
    double LLET = 250, LLE = 3, LSET = 90, LSE = 1;
    double ALET = 250, ALE = 3, ASET = 90, ASE = 1;

    // PID chaining thresholds
    double LCh = 3, ACh = 5, SCh = 3;

    // Slew rate limiting
    double LSSM = 80, LSDM = 3;
    double ASSM = 70, ASDM = 3;

    // Tracking wheel configuration
    int Vrt1Prt = 0, Vrt2Prt = 0, Hr1Prt = 0, Hr2Prt = 0;
    double Vrt1D = T_Lib::util::Wheel_Size::Omni_2, Vrt1DTC = 4;
    double Vrt2D = T_Lib::util::Wheel_Size::Omni_2, Vrt2DTC = 4;
    double Hr1D  = T_Lib::util::Wheel_Size::Omni_2, Hr1DTC = 4;
    double Hr2D  = T_Lib::util::Wheel_Size::Omni_2, Hr2DTC = 4;

    // Motor ID lists
    std::vector<int8_t> lemLeftMotors  = {LftMtr1, LftMtr2, LftMtr3, LftMtr4};
    std::vector<int8_t> lemRightMotors = {RtMtr1, RtMtr2, RtMtr3, RtMtr4};

    std::vector<int> ezLeftMotors  = {lemLeftMotors[0], lemLeftMotors[1], lemLeftMotors[2]};
    std::vector<int> ezRightMotors = {lemRightMotors[0], lemRightMotors[1], lemRightMotors[2]};

    // PROS MotorGroups
    pros::MotorGroup leftMotors(lemLeftMotors, Base_Color);
    pros::MotorGroup rightMotors(lemRightMotors, Base_Color);
}

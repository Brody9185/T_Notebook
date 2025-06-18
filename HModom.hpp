#include "main.h"
#include <cmath>
#include <cstddef>
#include <sys/types.h>

namespace Belt{
    namespace odom{
    std::vector<double> prevPos;
    double prevTheta;
    std::vector<double> currentPos;
    double currentTheta;
    double totalTheta;
    double changeTheta;
    double distance;
    double x_pos;
    double y_pos;
    inline Belt::Vert_Tracking_Wheel *LeftTracker;
    inline Belt::Vert_Tracking_Wheel *RightTracker;
    inline Belt::Hor_Tracking_Wheel *FrontTracker;
    inline Belt::Hor_Tracking_Wheel *BackTracker;
    inline pros::Imu *DriveImu1;
    inline pros::Imu *DriveImu2;
    inline void setLeftTrackingWheel(Vert_Tracking_Wheel LTracker){
        *LeftTracker = LTracker;
    }
    inline void setRightTrackingWheel(Vert_Tracking_Wheel RTracker){
        *RightTracker = RTracker;
    }

    // Updates the robot's position and orientation using available sensors
inline void compute() {
    // Case 1: Both Front and Back Trackers are available
    if (FrontTracker != nullptr && BackTracker != nullptr) {
        // Average the angle changes from both trackers
        totalTheta = (
            (((BackTracker->Change() / (std::numbers::pi * 2 * BackTracker->offset)) * 360) +
             ((FrontTracker->Change() / (std::numbers::pi * 2 * FrontTracker->offset)) * 360)) / 2);
        
        // Normalize the angle to [0, 360)
        currentTheta = std::fmod(totalTheta, 360.0);
        if (currentTheta < 0) currentTheta += 360.0;

    // Case 2: Only Front Tracker is available
    } else if (FrontTracker != nullptr) {
        totalTheta = ((FrontTracker->Change() / (std::numbers::pi * 2 * FrontTracker->offset)) * 360);
        currentTheta = std::fmod(totalTheta, 360.0);
        if (currentTheta < 0) currentTheta += 360.0;

    // Case 3: Only Back Tracker is available
    } else if (BackTracker != nullptr) {
        totalTheta = ((BackTracker->Change() / (std::numbers::pi * 2 * BackTracker->offset)) * 360);
        currentTheta = std::fmod(totalTheta, 360.0);
        if (currentTheta < 0) currentTheta += 360.0;

    // Case 4: Both IMUs are available
    } else if (DriveImu1 != nullptr && DriveImu2 != nullptr) {
        currentTheta = (DriveImu1->get_heading() + DriveImu2->get_heading()) / 2.0;

    // Case 5: Only one IMU available
    } else if (DriveImu1 != nullptr) {
        currentTheta = DriveImu1->get_heading();

    // Case 6: No trackers or IMUs — fallback to differential odometry
    } else {
        currentTheta = (RightTracker->Change() - LeftTracker->Change()) /
                       (std::abs(RightTracker->offset) + std::abs(LeftTracker->offset));
        currentTheta = std::fmod(currentTheta, 360.0);
        if (currentTheta < 0) currentTheta += 360.0;
    }

    // Calculate average distance traveled based on left and right trackers
    distance = (LeftTracker->Change() - RightTracker->Change()) / 2.0;

    // Convert currentTheta to radians for trigonometric calculations
    double thetaRad = currentTheta * (std::numbers::pi / 180.0);

    // Update position in X and Y axes
    x_pos += distance * std::cos(thetaRad);
    y_pos += distance * std::sin(thetaRad);
}
    }
}
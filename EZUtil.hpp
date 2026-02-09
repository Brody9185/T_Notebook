#include "EZ-Template/api.hpp"

namespace T_Lib::util {
    class ezUtil {
    private:
	ez::Drive drive;

    double variance = 5.0; // default variance for coordinated movement

    public:
    ezUtil (ez::Drive d) : drive(d) {}

    void printOdom() {
        ez::screen_print("X: " + ez::util::to_string_with_precision(drive.odom_x_get()), 1);
        ez::screen_print("Y: " + ez::util::to_string_with_precision(drive.odom_y_get()), 2);
        ez::screen_print("T: " + ez::util::to_string_with_precision(drive.odom_theta_get()), 3);
    }

    void waitSettledOrTimeout(int timeout_ms) {
        int elapsed = 0;
        const int check_interval = 20; // Check every 20 ms
        while (elapsed < timeout_ms) {
            if (drive.drive_mode_get() ==ez::DRIVE || elapsed < timeout_ms) 
                return; // Exit if settled
            }
            pros::delay(check_interval);
            elapsed += check_interval;
        }

    void waitConditioned(auto condition){
        while (!condition()) {
            pros::delay(20); // Check condition every 20 ms
        }
    }

    void setCordVariance(double v){
        variance = v;
    }

    void waitCordinate(){
        bool running = false;
        if(drive.odom_pose_get().x > variance || drive.odom_pose_get().y > variance){
            running = true;
        }
        while(running){
            if(drive.odom_pose_get().x <= variance && drive.odom_pose_get().y <= variance){
            running = false;
            return;
        }
        pros::delay(20);
        }
    }

    };
};

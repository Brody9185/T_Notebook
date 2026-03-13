#pragma once

#include "EZ-Template/drive/drive.hpp"
#include "pros/distance.hpp"
#include "pros/gps.hpp"
#include <cstdlib>
#include <cmath>
#include <vector>

namespace T_Lib {

enum side {
    left = 3,
    right = 1,
    front = 0,
    back = 2
};

enum dir {
    North = 1,
    East = 1,
    South = -1,
    West = -1
};

enum axis {
    x_axis = 0,
    y_axis = 1
};

    /**
     * Lightweight wrapper for a distance sensor object.
     * Stores the PROS object along with configuration used by Chassis.
     */
    class T_Distance {

    public:

    axis distance_axis;
    side SIDE;
    double x_offset;
    double y_offset;
    pros::Distance distance;
        T_Distance(int port, side s, double x, double y) : distance(port), SIDE(s), x_offset(x), y_offset(y) {
            if (SIDE == left || SIDE == right) {
                distance_axis = x_axis;
            } else if (SIDE == front || SIDE == back) {
                distance_axis = y_axis;
            }
        }

    void setAxis(axis a){
        distance_axis = a;
    }
};

    /**
     * GPS wrapper for use in the chassis class.  Simply holds the PROS
     * object and exposes a constructor matching the sensor port.
     */
    class T_GPS {
    public:
        pros::Gps gps;
        double x_offset = 0;
        double y_offset = 0;

        explicit T_GPS(int port) : gps(port) {}
        // position-initializing constructor
        T_GPS(int port, double xInit, double yInit, double headingInit)
            : gps(port, xInit, yInit, headingInit) {}
        // allow setting physical offset of sensor from robot center
        void set_offset(double xOff, double yOff) {
            x_offset = xOff;
            y_offset = yOff;
        }
    };

class T_Chassis{
    ez::Drive chassis;

    std::vector <T_Distance> distance_sensors;
    std::vector <T_GPS> gps_sensors;

    bool distCheck;
    bool gpsCheck;

    // configurable thresholds (in inches)
    double distanceThreshold = 48;
    double gpsMinRange = 20;   // default lower bound (inches)
    double gpsMaxRange = 120;  // default upper bound (inches)

    double dist_x;
    double dist_y;

    dir direction;


public:
    T_Chassis(ez::Drive c, bool distCheck, bool gpsCheck) : chassis(c), distCheck(distCheck), gpsCheck(gpsCheck) {}

    void add_gps_sensor(int port) {
        gps_sensors.emplace_back(port);
    }

    void add_gps_sensor(int port, double xInit, double yInit, double headingInit) {
        gps_sensors.emplace_back(port, xInit, yInit, headingInit);
    }

    /**
     * Set the physical distance-from-wall threshold for using a distance sensor.
     * A reading + its absolute offset must be less than this value to count.
     */
    void set_distance_threshold(double inches) {
        distanceThreshold = inches;
    }

    /**
     * Specify the valid radial range (in inches) for averaging GPS readings.
     * Sensors reporting outside [min,max] will be ignored.  Typical GPS dead
     * zone is roughly <1 m (39") or >3 m (118"), but adjust as needed.
     */
    void set_gps_range(double min_inches, double max_inches) {
        gpsMinRange = min_inches;
        gpsMaxRange = max_inches;
    }

    /**
     * Set the robot's starting position for both EZ odometry and any
     * configured GPS sensors.  Coordinates are in inches (convert as needed).
     */
    void set_starting_pos(double x, double y, double heading) {
        chassis.odom_x_set(x);
        chassis.odom_y_set(y);
        chassis.odom_theta_set(heading);
        if (gpsCheck) {
            for (auto &g : gps_sensors) {
                g.gps.set_position(x / 39.37, y / 39.37, heading); // GPS uses meters
            }
        }
    }

    void add_distance_sensor(int port, side s, double x, double y) {
        distance_sensors.emplace_back(port, s, x, y);
    }

    void findDir(){
        if(chassis.odom_theta_get() < 45 || chassis.odom_theta_get() > 315){
            //facing North
            direction = North;
        } else if(chassis.odom_theta_get() > 45 && chassis.odom_theta_get() < 135){
            //facing West
            direction = West;
        } else if(chassis.odom_theta_get() > 135 && chassis.odom_theta_get() < 225){
            //facing South
            direction = South;
        } else {
            //facing East
            direction = East;
        }
    }

    /**
     * Run the position correction routine.
     *
     * 
     * @param useDist    override the distance-sensor enable flag (default true)
     * @param useGps     override the GPS enable flag (default true)
     */
    void sensor_reset(bool useDist = true, bool useGps = true){
        bool effectiveDist = useDist && distCheck;
        bool effectiveGps = useGps && gpsCheck;

        if(effectiveDist){
            // accumulate values for averaging
            double sumX = 0;
            int countX = 0;
            double sumY = 0;
            int countY = 0;

            for(auto &sensor : distance_sensors){
                // Correct axis being updated based on robot orientation
                if (sensor.SIDE == left || sensor.SIDE == right){
                    if (direction == North || direction == South){
                        sensor.setAxis(x_axis);
                    } else if (direction == East || direction == West){
                        sensor.setAxis(y_axis);
                    }
                } else if (sensor.SIDE == front || sensor.SIDE == back){
                    if (direction == North || direction == South){
                        sensor.setAxis(y_axis);
                    } else if (direction == East || direction == West){
                        sensor.setAxis(x_axis);
                    }
                }

                // read the sensor value
                double measured = sensor.distance.get();
                // incorporate sensor offset when deciding if within threshold
                double effective = measured + std::abs(
                    sensor.distance_axis == x_axis ? sensor.x_offset : sensor.y_offset);
                if (effective < distanceThreshold) {
                    // compute val including offset for position reset
                    if (sensor.distance_axis == x_axis && (sensor.SIDE == left || sensor.SIDE == right)) {
                        double val;
                        if (sensor.SIDE == left) {
                            val = -72 + measured - sensor.x_offset;
                        } else {
                            val = 72 - measured - sensor.x_offset;
                        }
                        sumX += val;
                        countX++;
                    } else if (sensor.distance_axis == y_axis && (sensor.SIDE == front || sensor.SIDE == back)) {
                        double val;
                        if (sensor.SIDE == back) {
                            val = -72 + measured - sensor.y_offset;
                        } else {
                            val = 72 - measured - sensor.y_offset;
                        }
                        sumY += val;
                        countY++;
                    }
                }
            }

            // apply averaged positions if any
            if (countX > 0) {
                chassis.odom_x_set(sumX / countX);
            }
            if (countY > 0) {
                chassis.odom_y_set(sumY / countY);
            }
        }

        // if GPS is enabled, average the position from all gps sensors
        if (effectiveGps && !gps_sensors.empty()) {
            update_gps_position();
        }
    }

private:
    // helper used by sensor_reset and the GPS task
    void update_gps_position() {
        double sumXg = 0;
        double sumYg = 0;
        int countG = 0;
        for (auto &g : gps_sensors) {
            auto pos = g.gps.get_position();
            // convert meters to inches
            double xIn = pos.x * 39.37 - g.x_offset;
            double yIn = pos.y * 39.37 - g.y_offset;
            double dist = std::hypot(xIn, yIn);
            if (dist >= gpsMinRange && dist <= gpsMaxRange) {
                sumXg += xIn;
                sumYg += yIn;
                countG++;
            }
        }
        if (countG > 0) {
            chassis.odom_x_set(sumXg / countG);
            chassis.odom_y_set(sumYg / countG);
        }
    }

public:
    /**
     * Start a background pros::Task that continuously updates the robot's
     * odometry using GPS sensors.  The task will respect gpsCheck, gps range,
     * and offsets.  Returns the Task object so caller can store/kill as needed.
     */
    pros::Task start_gps_task() {
        return pros::Task([this](void*ptr){
            while (true) {
                if (gpsCheck) {
                    update_gps_position();
                }
                pros::delay(20);
            }
        });
    }
};

}
#pragma once
/*
#include "libs/include/EZ-Template/drive/drive.hpp"

#include "libs/include/Drive_Config.hpp"

#include "config.hpp"

namespace T_Lib :: AutoRouteCalc{
    inline std::vector<ez::pose> goals;
    inline std::vector<ez::pose> currentPos;
    inline std::vector<double> scores;
    inline int currentGoal = 0;
    inline int goalIndex = 0;

    inline void setGoalPos(std::vector<ez::pose> goalst){
        goals = goalst;
    }

    inline void addGoalPos(std::vector<ez::pose> goalsToAdd){
        goals.insert(goals.end(), goalsToAdd.begin(), goalsToAdd.end());
    }

    inline void saveCurrentPos(){
        currentPos.push_back(EZchassis.odom_pose_get());
    }

    inline void nextGoal(){
        if(currentGoal < goals.size()-1) currentGoal++;
    }

    inline void goToGoal(int targetGoal){
        if(targetGoal >= 0 && targetGoal < goals.size() - 1)currentGoal = targetGoal;
        else if(targetGoal < 0) currentGoal = 0;
        else if(targetGoal >= goals.size()) currentGoal = goals.size();
    }

    inline double distToGoal(){
        ez::pose current = EZchassis.odom_pose_get();
        return sqrt(pow(current.x - goals[currentGoal].x, 2) + pow(current.y - goals[currentGoal].y, 2));
    }

    inline double angleToGoal(){
        ez::pose current = EZchassis.odom_pose_get();
        return abs(goals[currentGoal].theta - current.theta);
    }

    inline double doubleBack(){
        ez::pose current = EZchassis.odom_pose_get();
        return (sqrt(pow(current.x - goals[currentGoal+1].x, 2) + pow(current.y - goals[currentGoal+1].y, 2)) - sqrt(pow(current.x - goals[currentGoal].x, 2) + pow(current.y - goals[currentGoal].y, 2)));
    }

    inline double timeTaken(){
        ez::pose current = EZchassis.odom_pose_get();
        double speed = CONFIG::Wheel_RPM * (CONFIG::Wheel_Diameter * 3.14159); //inches per second
        double time = (sqrt(pow(current.x - goals[currentGoal].x, 2) + pow(current.y - goals[currentGoal].y, 2))) / speed;
        return time;
    }

    inline double calcScore(){
        double score = 100;
        score -= distToGoal() * 0.5; //40% weight to distance
        score -= angleToGoal() * 0.4; //30% weight to angle
        score -= doubleBack() * 0.6; //20% weight to double back
        score -= timeTaken() * 0.5; //10% weight to time taken
        scores.push_back(score);
        return score;
    }

    inline void Index(){
        saveCurrentPos();
        calcScore();
        currentGoal += 1;
    }

    inline void reset(){
        goals.clear();
        currentPos.clear();
        scores.clear();
        currentGoal = 0;
        goalIndex = 0;
    }

    inline void printVals(){
        while(goalIndex < goals.size()){
        printf("Target Pos: (%.2f, %.2f, %.2f)\n", goals[goalIndex].x, goals[goalIndex].y, goals[goalIndex].theta);
        printf("Score: %f\n", scores[goalIndex]);
        goalIndex++;
    }
    }
}// namespace T_Lib :: AutoRouteCalc
*/
#pragma once

#include <Configuration.hpp>
#include <Geometry2d/Point.hpp>
#include <Pid.hpp>
#include <Geometry2d/Point.hpp>
#include <planning/SingleRobotPathPlanner.hpp>
#include <planning/RotationCommand.hpp>
#include <planning/MotionCommand.hpp>
#include <planning/MotionConstraints.hpp>
#include <planning/RotationConstraints.hpp>
class OurRobot;

/**
 * @brief Handles computer-side motion control
 * @details It is responsible for most of what gets sent out in a RadioTx
 *     packet. The MotionControl object is given an OurRobot at initialization
 *     and from then on will set the values in that robot's RadioTx packet
 *     directly whenever run() or stopped() is called.
 */

 struct MotionWrapper {
    Geometry2d::Point vel;
    float w;
 };
class MotionControl {
public:
    MotionControl();

    /**
     * Stops the robot.
     * The robot will decelerate at max acceleration until it stops.
     */
    // void stopped();

    /**
     * This runs PID control on the position and angle of the robot and
     * sets values in the robot's radioTx packet.
     */
    MotionWrapper run(std::unique_ptr<Planning::Path> path,
                                       std::unique_ptr<Planning::RotationCommand> rotationCommand,
                                       std::unique_ptr<Planning::MotionCommand> motionCommand,
                                       Geometry2d::Point pos,
                                       float angle,
                                       const MotionConstraints &motionConstraints,
                                       const RotationConstraints &rotationConstraints);

    static void createConfiguration(Configuration* cfg);

private:
    // sets the target velocity in the robot's radio packet
    // this method is used by both run() and stopped() and does the
    // velocity and acceleration limiting and conversion to robot velocity
    //"units"
    Geometry2d::Point _targetBodyVel(Geometry2d::Point targetVel);

    /// sets the target angle velocity in the robot's radio packet
    /// does velocity limiting and conversion to robot velocity "units"
    float _targetAngleVel(float angleVel);

    // OurRobot* _robot;

    /// The last velocity (in m/s, not the radioTx value) command that we sent
    /// to the robot
    Geometry2d::Point _lastVelCmd;

    /// the time in microseconds when the last velocity command was sent
    long _lastCmdTime;

    Pid _positionXController;
    Pid _positionYController;
    Pid _angleController;

    static ConfigDouble* _max_acceleration;
    static ConfigDouble* _max_velocity;
};

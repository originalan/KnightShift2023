package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * TO BE IMPLEMENTED
 * Used to store the current heading and position of the robot.
 * Specifically used for switching from Autonomous to Teleop, especially if we have Field-Centric Drive because of the initial IMU heading
 */
@Config
public class PoseStorage {

    public static Pose2d currentPose = new Pose2d();

}

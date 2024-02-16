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
    public static Pose2d startingAutoPose = new Pose2d();
    public static Pose2d currentPose = new Pose2d();
    public static double originalInitYaw = -1;
    public static double AUTO_SHIFT_DEGREES = 0;

    // backdrop april tag locations
    public static Pose2d blueLeftBackdrop = new Pose2d(60.75, 72 - 22.5 - 6.625);
    public static Pose2d blueMiddleBackdrop = new Pose2d(60.75, 72 - 22.5 - 12.75);;
    public static Pose2d blueRightBackdrop = new Pose2d(60.75, 72 - 22.5 - 18.75);;
    public static Pose2d redLeftBackdrop = new Pose2d(60.75, 72 + 22.5 + 18.75);;
    public static Pose2d redMiddleBackdrop = new Pose2d(60.75, 72 + 22.5 + 12.75);;
    public static Pose2d redRightBackdrop = new Pose2d(60.75, 72 + 22.5 + 6.625);;

}

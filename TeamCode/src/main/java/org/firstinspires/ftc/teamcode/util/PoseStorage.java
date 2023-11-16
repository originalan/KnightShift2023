package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseStorage {

    // all spike mark locations since I'm lazy
    public static Pose2d redLeftSideLeftSpikeMark = new Pose2d(-47.5,-36);
    public static Pose2d redLeftSideMiddleSpikeMark = new Pose2d(-36,-24.5);
    public static Pose2d redLeftSideRightSpikeMark = new Pose2d(-24.5,-36);
    public static Pose2d redRightSideLeftSpikeMark = new Pose2d(0.5,-36);
    public static Pose2d redRightSideMiddleSpikeMark = new Pose2d(12,-24.5);
    public static Pose2d redRightSideRightSpikeMark = new Pose2d(23.5,-36);
    public static Pose2d blueLeftSideLeftSpikeMark = new Pose2d(23.5,36);
    public static Pose2d blueLeftSideMiddleSpikeMark = new Pose2d(12,24.5);
    public static Pose2d blueLeftSideRightSpikeMark = new Pose2d(0.5,36);
    public static Pose2d blueRightSideLeftSpikeMark = new Pose2d(-24.5,36);
    public static Pose2d blueRightSideMiddleSpikeMark = new Pose2d(-36,24.5);
    public static Pose2d blueRightSideRightSpikeMark = new Pose2d(-47.5,36);

    // backdrop april tag locations
    public static Pose2d blueLeftBackdrop = new Pose2d(60.75, 72-22.5-6.625);
    public static Pose2d blueMiddleBackdrop = new Pose2d(60.75, 72-22.5-12.75);
    public static Pose2d blueRightBackdrop = new Pose2d(60.75, 72-22.5-18.75);
    public static Pose2d redLeftBackdrop = new Pose2d(60.75, -72+22.5+18.75);
    public static Pose2d redMiddleBackdrop = new Pose2d(60.75, -72+22.5+12.75);
    public static Pose2d redRightBackdrop = new Pose2d(60.75, -72+22.5+6.625);

    // white pixel stack locations
    public static Pose2d redOuterStack = new Pose2d(-72, -72+36);
    public static Pose2d redMiddleStack = new Pose2d(-72, -72+48);
    public static Pose2d redInnerStack = new Pose2d(-72, -72+60);
    public static Pose2d blueInnerStack = new Pose2d(-72, 72-60);
    public static Pose2d blueMiddleStack = new Pose2d(-72, 72-48);
    public static Pose2d blueOuterStack = new Pose2d(-72, 72-36);

}

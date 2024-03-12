package org.firstinspires.ftc.teamcode.settings;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmSettings {

    public static double CLAW_RIGHT_CLOSE = 0.31;
    public static double CLAW_RIGHT_OPEN = 0.14;
    public static double CLAW_LEFT_CLOSE = 0.34;
    public static double CLAW_LEFT_OPEN = 0.17;

    public static double ARM_PIVOT_SERVO_REST = 0;
    public static double ARM_PIVOT_SERVO_PIXELSTACK1 = 0.45; // redclose1, redclose2
    public static double ARM_PIVOT_SERVO_PIXELSTACK2 = 0.44; // redfar1
    public static double ARM_PIVOT_SERVO_GROUND = 0.455;
    public static double ARM_PIVOT_SERVO_YELLOW = 0;
    public static double ARM_PIVOT_TEST_POS = 0.455;

    public static int positionBottom = 0;
    public static int position1 = 580;
    public static int position2 = 545;
    public static int position3 = 510;
    public static int position4 = 475;
    public static int positionYellowPixel = 580;
    public static int positionPixelStack1 = (int)(5.0 / 360.0 * 1120) + positionBottom; // redclose1, redclose2
    public static int positionPixelStack2 = (int) (6.5 / 360.0 * 1120) + positionBottom; // redfar1

}

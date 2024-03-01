package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmSettings {

    public static double CLAW_RIGHT_CLOSE = 0.265;
    public static double CLAW_RIGHT_OPEN = 0;
    public static double CLAW_LEFT_CLOSE = 0.265;
    public static double CLAW_LEFT_OPEN = 0;

    public static double ARM_PIVOT_SERVO_REST = 0;
    public static double ARM_PIVOT_SERVO_PIXELSTACK1 = 0.45; // redclose1, redclose2
    public static double ARM_PIVOT_SERVO_PIXELSTACK2 = 0.44; // redfar1
    public static double ARM_PIVOT_SERVO_GROUND = 0.45;
    public static double ARM_PIVOT_SERVO_YELLOW = 0;
    public static double ARM_PIVOT_TEST_POS = 0.43;

    public static int positionBottom = 0;
    public static int position1 = 500;
    public static int position2 = 550;
    public static int position3 = 600;
    public static int positionYellowPixel = 600;
    public static int positionPixelStack1 = (int)(5.0 / 360.0 * 1120) + positionBottom; // redclose1, redclose2
    public static int positionPixelStack2 = (int) (6.5 / 360.0 * 1120) + positionBottom; // redfar1
    public static int ENCODER_TICKS_PER_SECOND = 25; // for override feature in twodriver

}

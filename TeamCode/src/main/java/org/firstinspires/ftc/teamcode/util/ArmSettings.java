package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmSettings {

    public static double CLAW_RIGHT_CLOSE = 0.265;
    public static double CLAW_RIGHT_OPEN = 0;
    public static double CLAW_LEFT_CLOSE = 0.265;
    public static double CLAW_LEFT_OPEN = 0;

    public static double ARM_PIVOT_SERVO_REST = 0;
    public static double ARM_PIVOT_SERVO_PIXELSTACK1 = 0.08; // redclose1, redclose2
    public static double ARM_PIVOT_SERVO_PIXELSTACK2 = 0.07; // redfar1
    public static double ARM_PIVOT_SERVO_GROUND = 0.5;
    public static double ARM_PIVOT_SERVO_YELLOW = 0.9444;
    public static double ARM_PIVOT_TEST_POS = 0.5;

    public static int positionBottom = 3;
    public static int position1 = 50;
    public static int position2 = 100;
    public static int position3 = 150;
    public static int positionYellowPixel = (int)(200.0 / 360.0 * 1120) + positionBottom;
    public static int positionPixelStack1 = (int)(5.0 / 360.0 * 1120) + positionBottom; // redclose1, redclose2
    public static int positionPixelStack2 = (int) (6.5 / 360.0 * 1120) + positionBottom; // redfar1

}

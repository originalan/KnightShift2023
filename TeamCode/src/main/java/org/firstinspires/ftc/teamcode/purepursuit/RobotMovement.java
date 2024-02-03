package org.firstinspires.ftc.teamcode.purepursuit;

import com.qualcomm.robotcore.util.Range;

public class RobotMovement {

    private static double worldX = 50, worldY = 50, worldAngle_rad = Math.toRadians(180);

    /**
     *
     * @param x is the x of the point to go to
     * @param y is the y of the point to go to
     * @param movementSpeed is the speed of motors
     * @param preferredAngle is the preferred direction you want the robot to move in
     * @param turnSpeed is the speed at which the robot will turn
     */
    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {

        // WorldX, WorldY, WorldAngle are the values of the robot
        double distanceToTarget = Math.hypot(x - worldX, y - worldY);

        double absoluteAngleToPoint = Math.atan2(y - worldY, x - worldX);

        double relativeAngleToPoint = absoluteAngleToPoint - MathFunctions.angleWrap(worldAngle_rad - Math.toRadians(90)); // can delete the 90

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        // Normalize this vector
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        MovementVars.movement_x = movementXPower * movementSpeed;
        MovementVars.movement_y = movementYPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        MovementVars.movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;

        // To stop the weird orbiting behavior when near the point
        if (distanceToTarget < 10) {
            MovementVars.movement_turn = 0;
        }

    }

}

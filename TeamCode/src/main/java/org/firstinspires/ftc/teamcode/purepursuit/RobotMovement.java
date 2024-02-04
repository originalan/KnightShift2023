package org.firstinspires.ftc.teamcode.purepursuit;

import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Point;

import java.util.ArrayList;

public class RobotMovement {

    private static double worldX = 50, worldY = 50, worldAngle_rad = Math.toRadians(180);

    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle) {



        CurvePoint followMe = getFollowPointPath(allPoints, new Point(worldX, worldY),
                allPoints.get(0).followDistance); // follow distance is best not consistent for best pure pursuit

        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);
    }

    /**
     * CORE algorithm of pure pursuit, following the instantaneous vector / point
     * @param pathPoints
     * @param robotLocation location of robot (x,y)
     * @param followRadius of robot
     * @return
     */
    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius) {

        // Default: go to first point in arraylist
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for (int i = 0; i < pathPoints.size() - 1; i++) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i+1);

            ArrayList<Point> intersections = MathFunctions.lineCircleIntersection(robotLocation, followRadius,
                    startLine.toPoint(), endLine.toPoint());

            double closestAngle = 10000000;
            for (Point thisIntersection : intersections) {
                double angle = Math.atan2(thisIntersection.y - worldY, thisIntersection.x - worldX);
                double deltaAngle = Math.abs(MathFunctions.angleWrap(angle - worldAngle_rad));

                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }

        }
        return followMe;

    }

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

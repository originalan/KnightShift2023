package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MotionProfile {
    private double startingTime = 0;

    public static int MAX_VELOCITY = 250; // enocder ticks per second
    public static int MAX_ACCELERATION = 250; // encoder ticks per second
//    public static int MAX_JERK = 0;

    private double acceleration_dt, distance, halfway_distance, max_acceleration,
            max_velocity, acceleration_distance, deceleration_dt,
            cruise_distance, cruise_dt, deceleration_time, entire_dt,
            start, end;

    private double instantPos = 0;
    private double instantVel = 0;
    private double instantAcl = 0;
    private boolean isBusy = false;

//    public double profileTime = 100; // seconds

    public MotionProfile() { }
    public void setProfile(double start, double end) {
        isBusy = true;

        this.start = start;
        this.end = end;
        distance = end - start;
        max_acceleration = MAX_ACCELERATION;
        max_velocity = MAX_VELOCITY;

        if (distance == 0 || max_acceleration == 0 || max_velocity == 0) {
            isBusy = false;
            instantPos = end;
            instantVel = 0;
            instantAcl = 0;
//            return;
        }

        if (distance < 0) {
            max_acceleration *= -1;
            max_velocity *= -1;
        }

        // Calculate the time it takes to accelerate to max velocity
        acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        halfway_distance = distance / 2;
        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        if (Math.abs(acceleration_distance) > Math.abs(halfway_distance)) {
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));
        }

        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        deceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        cruise_distance = distance - 2 * acceleration_distance;
        cruise_dt = cruise_distance / max_velocity;
        deceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        entire_dt = acceleration_dt + cruise_dt + deceleration_dt;

        // for back and forth test opmode
//        profileTime = entire_dt;
    }

    public void updateState(double currentTime) {
        isBusy = true;
        double timeElapsed = currentTime - startingTime;
        timeElapsed = Math.abs(timeElapsed);


        if (distance == 0 || max_acceleration == 0 || max_velocity == 0) {
            isBusy = false;
            instantPos = end;
            instantVel = 0;
            instantAcl = 0;
            return;
        }

        // if motion profile is done
        if (timeElapsed > entire_dt) {
            isBusy = false;
            instantPos = end;
            instantVel = 0;
            instantAcl = 0;
            return;
        }

        // if we're accelerating
        if (timeElapsed < acceleration_dt) {
            // use the kinematic equation for acceleration
            instantPos = 0.5 * max_acceleration * Math.pow(timeElapsed, 2);
            instantPos += start;
            instantVel = max_acceleration * timeElapsed;
            instantAcl = max_acceleration;
            return;
        }

        // if we're cruising
        else if (timeElapsed < deceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            double cruise_current_dt = timeElapsed - acceleration_dt;

            // use the kinematic equation for constant velocity
            instantPos = acceleration_distance + max_velocity * cruise_current_dt;
            instantPos += start;
            instantVel = max_velocity;
            instantAcl = 0;
            return;
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            cruise_distance = max_velocity * cruise_dt;
            double decelerationTime = timeElapsed - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            instantPos = acceleration_distance + cruise_distance + max_velocity * decelerationTime - 0.5 * max_acceleration * Math.pow(decelerationTime, 2);
            instantPos += start;
            instantVel = max_velocity - max_acceleration * (timeElapsed - deceleration_time);
            instantAcl = -1.0 * max_acceleration;
            return;
        }
    }
    public void setStartingTime(double startingTime) {
        this.startingTime = startingTime;
    }
    public double getInstantPosition() {
        return instantPos;
    }
    public double getInstantVelocity() {
        return instantVel;
    }
    public double getInstantAcceleration() {
        return instantAcl;
    }
    public boolean isBusy() {
        return isBusy;
    }
}

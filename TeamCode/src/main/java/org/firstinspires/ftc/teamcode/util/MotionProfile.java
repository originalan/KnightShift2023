package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MotionProfile {

    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();
    private double currentTime = 0;

    public static int MAX_VELOCITY = 250;
    public static int MAX_ACCELERATION = 175;

    public MotionProfile() {

    }

    public void setTime(double time) {
        currentTime = time;
    }

    /**
     * Returns current reference position based on given distance and current time
     * @param distance
     * @param elapsed_time
     * @return
     */
    public double motionProfile(double distance, double elapsed_time) {
        /*
        Return the current reference position based on the given motion profile times, maximum acceleration, velocity, and current time.
        */
        double max_acceleration = MAX_ACCELERATION;
        double max_velocity = MAX_VELOCITY;

        if (distance == 0 || max_acceleration == 0 || max_velocity == 0) {
            return 0;
        }
        if (distance < 0) {
            max_acceleration *= -1;
            max_velocity *= -1;
        }

        // Calculate the time it takes to accelerate to max velocity
        double acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = distance / 2;
        double acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        if (Math.abs(acceleration_distance) > Math.abs(halfway_distance)) {
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));
        }

        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        double deceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        double cruise_distance = distance - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / max_velocity;
        double deceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        if (elapsed_time > entire_dt) {
            return distance;
        }

        // if we're accelerating
        if (elapsed_time < acceleration_dt) {
            // use the kinematic equation for acceleration
            return 0.5 * max_acceleration * Math.pow(elapsed_time, 2);
        }

        // if we're cruising
        else if (elapsed_time < deceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            double cruise_current_dt = elapsed_time - acceleration_dt;

            // use the kinematic equation for constant velocity
            return acceleration_distance + max_velocity * cruise_current_dt;
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            cruise_distance = max_velocity * cruise_dt;
            deceleration_time = elapsed_time - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * Math.pow(deceleration_time, 2);
        }
    }

    /**
     * Returns current goal position based on
     * @param start
     * @param end
     * @precondition call setTime() method before this
     * @return
     */
    public double position(double start, double end) {
        double distance = end - start;
        double max_acceleration = MAX_ACCELERATION;
        double max_velocity = MAX_VELOCITY;

        if (distance == 0 || max_acceleration == 0 || max_velocity == 0) {
            return 0;
        }

        return 0;
    }

    public double velocity() {
        return 0;
    }

}

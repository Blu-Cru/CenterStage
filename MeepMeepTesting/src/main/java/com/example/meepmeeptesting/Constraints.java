package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import java.util.Arrays;

public class Constraints {
    public static TrajectoryVelocityConstraint FAST_VEL = new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(Math.toRadians(200)),
                new MecanumVelocityConstraint(35, 12.6)));

    public static TrajectoryVelocityConstraint NORMAL_VEL = new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(Math.toRadians(180)),
            new MecanumVelocityConstraint(30, 12.6)));

    public static TrajectoryVelocityConstraint SLOW_VEL = new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(Math.toRadians(150)),
            new MecanumVelocityConstraint(15, 12.6)));

    public static TrajectoryAccelerationConstraint FAST_ACCEL =  new ProfileAccelerationConstraint(35);
    public static TrajectoryAccelerationConstraint NORMAL_ACCEL =  new ProfileAccelerationConstraint(30);
    public static TrajectoryAccelerationConstraint SLOW_ACCEL =  new ProfileAccelerationConstraint(20);
}

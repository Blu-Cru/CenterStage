package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import java.util.Arrays;

public class Constraints {
    public static TrajectoryVelocityConstraint FAST_VELOCITY = new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(Math.toRadians(360)),
                new MecanumVelocityConstraint(45, 12.6)));

    public static TrajectoryVelocityConstraint NORMAL_VELOCITY = new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(Math.toRadians(270)),
            new MecanumVelocityConstraint(30, 12.6)));

    public static TrajectoryVelocityConstraint SLOW_VELOCITY = new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(Math.toRadians(180)),
            new MecanumVelocityConstraint(15, 12.6)));

    public static TrajectoryAccelerationConstraint FAST_ACCELERATION =  new ProfileAccelerationConstraint(50);
    public static TrajectoryAccelerationConstraint NORMAL_ACCELERATION =  new ProfileAccelerationConstraint(35);
    public static TrajectoryAccelerationConstraint SLOW_ACCELERATION =  new ProfileAccelerationConstraint(35);
}

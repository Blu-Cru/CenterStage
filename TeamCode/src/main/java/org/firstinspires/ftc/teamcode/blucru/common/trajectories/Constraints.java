package org.firstinspires.ftc.teamcode.blucru.common.trajectories;

import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Constraints {
    public static TrajectoryVelocityConstraint FAST_VELOCITY = SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(180), DriveConstants.TRACK_WIDTH);
    public static TrajectoryVelocityConstraint NORMAL_VELOCITY = SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(180), DriveConstants.TRACK_WIDTH);
    public static TrajectoryVelocityConstraint SLOW_VELOCITY = SampleMecanumDrive.getVelocityConstraint(15, Math.toRadians(180), DriveConstants.TRACK_WIDTH);

    public static TrajectoryAccelerationConstraint FAST_ACCELERATION = SampleMecanumDrive.getAccelerationConstraint(35);
    public static TrajectoryAccelerationConstraint NORMAL_ACCELERATION = SampleMecanumDrive.getAccelerationConstraint(30);
    public static TrajectoryAccelerationConstraint SLOW_ACCELERATION = SampleMecanumDrive.getAccelerationConstraint(20);
}

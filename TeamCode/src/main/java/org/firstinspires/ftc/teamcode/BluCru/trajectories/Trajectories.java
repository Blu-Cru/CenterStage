package org.firstinspires.ftc.teamcode.BluCru.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.geometry.partitioning.Side;
import org.firstinspires.ftc.teamcode.BluCru.states.Alliance;
import org.firstinspires.ftc.teamcode.BluCru.states.StartSide;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Trajectories {
    private static final Pose2d rightStartingPose = new Pose2d(10, -61, Math.toRadians(-90));
    private static final Pose2d leftStartingPose = new Pose2d(-34, -61, Math.toRadians(-90));
    private static final Pose2d placementLeftPose = new Pose2d(10, -59, Math.toRadians(-30));
    private static final Pose2d placementCenterPose = new Pose2d(10, -38, Math.toRadians(-120));
    private static final Pose2d placementRightPose = new Pose2d(10, -17);
    private static final Pose2d swervePlacementLeftPose = new Pose2d(10, -61, Math.toRadians(-90));
    private static final Pose2d swervePlacementRightPose = new Pose2d(10, -61, Math.toRadians(-90));

    Alliance alliance;
    StartSide side;
    Robot robot;

    public Trajectories(Alliance alliance, StartSide side, Robot robot) {
        this.alliance = alliance;
        this.side = side;
        this.robot = robot;

    }

    public TrajectorySequence placement(int position) {
        return robot.drivetrain.trajectorySequenceBuilder(rightStartingPose)
                .build();
    }
}

package org.firstinspires.ftc.teamcode.blucru.common.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Placements {
    public static double reflect = 1;

    public static double RELEASE_TIME = 0;
    public static double WAIT_TIME = 0.1;

    public static double DROP_TIME = 1;

    public Placements(double reflect) {
        this.reflect = reflect;
    }

    public TrajectorySequence placementBackdropClose(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.BACKDROP_STARTING_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(90 * reflect))
                .lineTo(Poses.BACKDROP_PLACEMENT_CLOSE_POSE.vec())
                // release purple pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> robot.purplePixelHolder.retract())
                .waitSeconds(WAIT_TIME)
                .build();
    }

    public TrajectorySequence placementBackdropCenter(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.BACKDROP_STARTING_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(90 * reflect))
                .splineToConstantHeading(new Vector2d(12, -50 * reflect), Math.toRadians(90 * reflect))
                .splineToConstantHeading(Poses.BACKDROP_PLACEMENT_CENTER_POSE.vec(), Math.toRadians(90 * reflect))
                // release purple pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> robot.purplePixelHolder.retract())
                .waitSeconds(WAIT_TIME)
                .build();
    }

    public TrajectorySequence placementBackdropFar(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.BACKDROP_STARTING_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(90 * reflect))
                .splineTo(new Vector2d(12, -55 * reflect), Math.toRadians(90 * reflect))
                .splineToSplineHeading(Poses.BACKDROP_PLACEMENT_FAR_POSE, Math.toRadians(135 * reflect))
                // release purple pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> robot.purplePixelHolder.retract())
                .waitSeconds(WAIT_TIME)
                .build();
    }

    public TrajectorySequence placementWingCloseForCenter(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.WING_STARTING_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(90 * reflect))
                // drop down ready
                .UNSTABLE_addTemporalMarkerOffset(DROP_TIME, () -> robot.intake.setIntakeWristTargetHeight(Intake.WRIST_AUTO_READY_HEIGHT))
                .splineToConstantHeading(new Vector2d(-36, -52 * reflect), Math.toRadians(90 * reflect))
                .splineToSplineHeading(Poses.WING_PLACEMENT_CLOSE_FOR_CENTER_POSE, Math.toRadians(45 * reflect))
                // release purple pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> robot.purplePixelHolder.retract())
                .waitSeconds(WAIT_TIME)
                .build();
    }

    public TrajectorySequence placementWingCloseForPerimeter(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.WING_STARTING_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(90 * reflect))
                // drop down ready
                .UNSTABLE_addTemporalMarkerOffset(DROP_TIME, () -> robot.intake.setIntakeWristTargetHeight(Intake.WRIST_AUTO_READY_HEIGHT))
                .splineToConstantHeading(new Vector2d(-36, -50 * reflect), Math.toRadians(90 * reflect))
                .splineToSplineHeading(Poses.WING_PLACEMENT_CLOSE_FOR_PERIM_POSE, Math.toRadians(45 * reflect))
                // release purple pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> robot.purplePixelHolder.retract())
                .waitSeconds(WAIT_TIME)
                .build();
    }

    public TrajectorySequence placementWingCenter(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.WING_STARTING_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(90))
                // drop down ready
                .UNSTABLE_addTemporalMarkerOffset(DROP_TIME, () -> robot.intake.setIntakeWristTargetHeight(Intake.WRIST_AUTO_READY_HEIGHT))
                .splineToConstantHeading(new Vector2d(-36, -59*reflect), Math.toRadians(90))
                .splineToSplineHeading(Poses.WING_PLACEMENT_CENTER_POSE, Math.toRadians(90))
                // release purple pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> robot.purplePixelHolder.retract())
                .waitSeconds(WAIT_TIME)
                .build();
    }

    public TrajectorySequence placementWingFarForPerimeter(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.WING_STARTING_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                // placement
                .setTangent(Math.toRadians(90 * reflect))
                // drop down ready
                .UNSTABLE_addTemporalMarkerOffset(DROP_TIME, () -> robot.intake.setIntakeWristTargetHeight(Intake.WRIST_AUTO_READY_HEIGHT))
                .splineTo(new Vector2d(-36, -55*reflect), Math.toRadians(90 * reflect))
                .splineToSplineHeading(Poses.WING_PLACEMENT_FAR_FOR_PERIM_POSE, Math.toRadians(180))
                // release purple pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> robot.purplePixelHolder.retract())
                // drop down and start intake

                .waitSeconds(WAIT_TIME)
                .build();
    }

    public TrajectorySequence placementWingFarForCenter(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.WING_STARTING_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                // placement
                .setTangent(Math.toRadians(130 * reflect))
                // drop down ready
                .UNSTABLE_addTemporalMarkerOffset(DROP_TIME, () -> robot.intake.setIntakeWristTargetHeight(Intake.WRIST_AUTO_READY_HEIGHT))
                .splineToConstantHeading(new Vector2d(-46, -55*reflect), Math.toRadians(90 * reflect))
                .splineToSplineHeading(new Pose2d(-46, -40 * reflect, Math.toRadians(180 * reflect)), Math.toRadians(90 * reflect))
                .splineToConstantHeading(new Vector2d(-46, -10 * reflect), Math.toRadians(90 * reflect))
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .lineToLinearHeading(Poses.WING_PLACEMENT_FAR_FOR_CENTER_POSE)
                // release purple pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> robot.purplePixelHolder.retract())
                // drop down and start intake

                .waitSeconds(WAIT_TIME)
                .build();
    }
}

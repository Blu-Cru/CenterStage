package org.firstinspires.ftc.teamcode.blucru.common.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

// class for purple pixel placements
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
                .setVelConstraint(Constraints.FAST_VEL)
                .setTangent(Math.toRadians(90 * reflect))
                .lineTo(Poses.BACKDROP_PLACEMENT_CLOSE_POSE.vec())
                // release purple pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> robot.purplePixelHolder.retractRight())
                .waitSeconds(WAIT_TIME)
                .build();
    }

    public TrajectorySequence placementBackdropCenter(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.BACKDROP_STARTING_POSE)
                .setVelConstraint(Constraints.FAST_VEL)
                .setTangent(Math.toRadians(90 * reflect))
                .splineToConstantHeading(new Vector2d(12, -50 * reflect), Math.toRadians(90 * reflect))
                .splineToConstantHeading(Poses.BACKDROP_PLACEMENT_CENTER_POSE.vec(), Math.toRadians(90 * reflect))
                // release purple pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> robot.purplePixelHolder.retractRight())
                .waitSeconds(WAIT_TIME)
                .build();
    }

    public TrajectorySequence placementBackdropFar(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.BACKDROP_STARTING_POSE)
                .setVelConstraint(Constraints.FAST_VEL)
                .setTangent(Math.toRadians(90 * reflect))
                .splineTo(new Vector2d(12, -55 * reflect), Math.toRadians(90 * reflect))
                .splineToSplineHeading(Poses.BACKDROP_PLACEMENT_FAR_POSE, Math.toRadians(135 * reflect))
                // release purple pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> robot.purplePixelHolder.retractRight())
                .waitSeconds(WAIT_TIME)
                .build();
    }

    public TrajectorySequence placementWingCloseForCenter(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.AUDIENCE_STARTING_POSE)
                .setVelConstraint(Constraints.NORMAL_VEL)
                .setTangent(Math.toRadians(90 * reflect))
                // drop down ready
                .UNSTABLE_addTemporalMarkerOffset(DROP_TIME, () -> robot.intake.dropdown.dropToAutoMidPos())
                .splineToConstantHeading(new Vector2d(-36 + Poses.FIELD_OFFSET_X, -52 * reflect), Math.toRadians(90 * reflect))
                .splineToSplineHeading(Poses.WING_PLACEMENT_CLOSE_FOR_CENTER_POSE, Math.toRadians(45 * reflect))
                // release purple pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> robot.purplePixelHolder.retractRight())
                .waitSeconds(WAIT_TIME)
                .build();
    }

    public TrajectorySequence placementWingCloseForPerimeter(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.AUDIENCE_STARTING_POSE)
                .setVelConstraint(Constraints.NORMAL_VEL)
                .setTangent(Math.toRadians(90 * reflect))
                // drop down ready
                .UNSTABLE_addTemporalMarkerOffset(DROP_TIME, () -> robot.intake.dropdown.dropToAutoMidPos())
                .splineToConstantHeading(new Vector2d(-36 + Poses.FIELD_OFFSET_X, -50 * reflect), Math.toRadians(90 * reflect))
                .splineToSplineHeading(Poses.WING_PLACEMENT_CLOSE_FOR_PERIM_POSE, Math.toRadians(45 * reflect))
                // release purple pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> robot.purplePixelHolder.retractRight())
                .waitSeconds(WAIT_TIME)
                .build();
    }

    public TrajectorySequence placementWingCenter(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.AUDIENCE_STARTING_POSE)
                .setVelConstraint(Constraints.NORMAL_VEL)
                .setTangent(Math.toRadians(90 * reflect))
                // drop down ready
                .UNSTABLE_addTemporalMarkerOffset(DROP_TIME, () -> robot.intake.dropdown.dropToAutoMidPos())
                .splineToConstantHeading(new Vector2d(-45 + Poses.FIELD_OFFSET_X, -40*reflect), Math.toRadians(90 * reflect))
                .splineToSplineHeading(Poses.WING_PLACEMENT_CENTER_POSE, Math.toRadians(135 * reflect))
                // release purple pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> robot.purplePixelHolder.retractRight())
                .waitSeconds(WAIT_TIME)
                .build();
    }

    public TrajectorySequence placementWingFarForPerimeter(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.AUDIENCE_STARTING_POSE)
                .setVelConstraint(Constraints.NORMAL_VEL)
                // placement
                .setTangent(Math.toRadians(90 * reflect))
                // drop down ready
                .UNSTABLE_addTemporalMarkerOffset(DROP_TIME, () -> robot.intake.dropdown.dropToAutoMidPos())
                .splineTo(new Vector2d(-36 + Poses.FIELD_OFFSET_X, -55*reflect), Math.toRadians(90 * reflect))
                .splineToSplineHeading(Poses.WING_PLACEMENT_FAR_FOR_PERIM_POSE, Math.toRadians(180))
                // release purple pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> robot.purplePixelHolder.retractRight())
                // drop down and start intake

                .waitSeconds(WAIT_TIME)
                .build();
    }

    public TrajectorySequence placementWingFarForCenter(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.AUDIENCE_STARTING_POSE)
                .setVelConstraint(Constraints.NORMAL_VEL)
                // placement
                .setTangent(Math.toRadians(130 * reflect))
                // drop down ready
                .UNSTABLE_addTemporalMarkerOffset(DROP_TIME, () -> robot.intake.dropdown.dropToAutoMidPos())
                .splineToConstantHeading(new Vector2d(-50 + Poses.FIELD_OFFSET_X, -50*reflect), Math.toRadians(90 * reflect))
                .splineToSplineHeading(new Pose2d(-54.5 + Poses.FIELD_OFFSET_X, -35 * reflect, Math.toRadians(180 * reflect)), Math.toRadians(90 * reflect))
                .splineToSplineHeading(Poses.WING_PLACEMENT_FAR_FOR_CENTER_POSE, Math.toRadians(70 * reflect))
                // release purple pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> robot.purplePixelHolder.retractRight())
                .waitSeconds(WAIT_TIME)
                .build();
    }
}

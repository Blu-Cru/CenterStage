package org.firstinspires.ftc.teamcode.blucru.common.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class IntakeTrajectories {
    public static double
            DROP_INTAKE_TIME = -1.0,
            START_INTAKE_TIME = -0.5,
            CENTER_TURRET_TIME = 0.1,
            WRIST_RETRACT_TIME = 0.4,
            LIFT_RETRACT_TIME = 0.5,
            INTAKE_LENGTH = 0.0,
            INTAKE_HYPOTENUSE = INTAKE_LENGTH * INTAKE_LENGTH,

            INTAKE_X = -53 + Poses.FIELD_OFFSET_X,
            INTAKE_SETUP_X = INTAKE_X + 2;

    double reflect;

    public IntakeTrajectories(double reflect) {
        this.reflect = reflect;
    }

    public TrajectorySequence intakeCenterStrafing(Robot robot, int stackHeight, double strafeDistance, double strafeAngleDeg, Pose2d startingPose) {
        return robot.drivetrain.trajectorySequenceBuilder(startingPose)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(135 * reflect))

                // retract turret
                .UNSTABLE_addTemporalMarkerOffset(CENTER_TURRET_TIME, () -> robot.outtake.centerTurret())
                // retract wrist
                .UNSTABLE_addTemporalMarkerOffset(WRIST_RETRACT_TIME, () -> robot.outtake.wristRetract())
                // retract lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_RETRACT_TIME, () -> {
                    robot.outtake.retractLift();
                    robot.intake.dropdown.dropToAutoMidPos();
                })

                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-25 + Poses.FIELD_OFFSET_X, -12 * reflect), Math.toRadians(180))

                .splineToSplineHeading(new Pose2d(INTAKE_SETUP_X + Poses.FIELD_OFFSET_X, -8 * reflect, Math.toRadians(180 + strafeAngleDeg * reflect)), Math.toRadians(180))

                .UNSTABLE_addTemporalMarkerOffset(-1.4, () -> {
                    robot.intake.dropToStack(stackHeight);
                    robot.intake.intake();
                    robot.outtake.unlock();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    robot.intake.dropToGround();
                })
                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.STACK_X + Poses.FIELD_OFFSET_X, (-12 - strafeDistance) * reflect), Math.toRadians(270 * reflect))
                .waitSeconds(1.3)
                .build();
    }

    public TrajectorySequence placePurpleIntakeFromWingCenter(Robot robot) {
//        Pose2d endPose = new Pose2d(Poses.STACK_X - Dropdown.RADIUS + Dropdown.toX(4), -12 * reflect, Math.toRadians(180*reflect));

        return robot.drivetrain.trajectorySequenceBuilder(Poses.AUDIENCE_STARTING_POSE)
                .setTangent(Math.toRadians(90 * reflect))
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .splineToConstantHeading(new Vector2d(-36 + Poses.FIELD_OFFSET_X, -58 * reflect), Math.toRadians(90 * reflect))
                .splineToSplineHeading(new Pose2d(-48 + Poses.FIELD_OFFSET_X, -25 * reflect, Math.toRadians(180 * reflect)), Math.toRadians(135*reflect))
                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                    robot.purplePixelHolder.release(reflect);
                })
                .splineToConstantHeading(new Vector2d(-49 + Poses.FIELD_OFFSET_X, -23 * reflect), Math.toRadians(140*reflect))
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.outtake.unlock();
                    robot.intake.intake();
                    robot.intake.dropToStack(4);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.intake.dropToStack(3);
                })
//                .splineToConstantHeading(endPose.vec(), Math.toRadians(130*reflect))
//                .addTemporalMarker(() -> robot.drivetrain.pidTo(endPose))
                .waitSeconds(5)
                .build();
    }

    public TrajectorySequence intakeCenterStraight(Robot robot, int stackHeight, Pose2d startingPose) {
        return robot.drivetrain.trajectorySequenceBuilder(startingPose)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(135 * reflect))

                // retract turret
                .UNSTABLE_addTemporalMarkerOffset(CENTER_TURRET_TIME, () -> robot.outtake.centerTurret())
                // retract wrist
                .UNSTABLE_addTemporalMarkerOffset(WRIST_RETRACT_TIME, () -> robot.outtake.wristRetract())
                // retract lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_RETRACT_TIME, () -> {
                    robot.outtake.retractLift();
                    robot.intake.dropdown.dropToAutoMidPos();
                })

                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-30 + Poses.FIELD_OFFSET_X, -12 * reflect), Math.toRadians(180))
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .UNSTABLE_addTemporalMarkerOffset(-1.4, () -> {
                    robot.intake.dropToStack(stackHeight + 1);
                    robot.intake.intake();
                    robot.outtake.unlock();
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.dropToStack(stackHeight);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.intake.dropToGround();
                })
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
//                .splineToConstantHeading(new Vector2d(Poses.calculateStackX(stackHeight), -12 * reflect), Math.toRadians(180 * reflect))
                .waitSeconds(3)
                .build();
    }

    public TrajectorySequence placePurpleIntakeThroughCenterFromBackdropClose(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_CLOSE_POSE)
                .setTangent(Math.toRadians(135 * reflect))
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                // retract turret
                .UNSTABLE_addTemporalMarkerOffset(CENTER_TURRET_TIME, () -> robot.outtake.centerTurret())
                // retract wrist
                .UNSTABLE_addTemporalMarkerOffset(WRIST_RETRACT_TIME, () -> robot.outtake.wristRetract())
                // retract lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_RETRACT_TIME, () -> robot.outtake.lift.setMotionProfileTargetPos(0))

                // drop down

                .splineToSplineHeading(new Pose2d(20, -20 * reflect, Math.toRadians(135 * reflect)), Math.toRadians(180*reflect))
                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                // release purple pixel
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.purplePixelHolder.release(reflect))

                .splineToConstantHeading(new Vector2d(17, -20 * reflect), Math.toRadians(135 * reflect))
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .splineToSplineHeading(new Pose2d(6, -12 * reflect, Math.toRadians(180)), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X, -12 * reflect), Math.toRadians(180))
                .build();
    }
}

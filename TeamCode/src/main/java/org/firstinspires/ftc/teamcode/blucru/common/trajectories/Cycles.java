package org.firstinspires.ftc.teamcode.blucru.common.trajectories;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Cycles {
    public static double CENTER_TURRET_TIME = 0;
    public static double WRIST_RETRACT_TIME = 0.3;
    public static double LIFT_RETRACT_TIME = 0.4;
    public static double LIFT_TIME = -1.1;
    public static double WRIST_EXTEND_TIME = -0.85;
    public static double TURN_TURRET_TIME = -0.4;

    public static double INTAKE_READY_TIME = -3.5;
    public static double START_INTAKE_TIME = -1.5;
    public static double DROP_INTAKE_1_TIME = -0.5;
    public static double DROP_INTAKE_2_TIME = 0;

    public static double TOTAL_DEPOSIT_TIME = 0.3;
    public static double INTAKE_1_TIME = 0.7;
    public static double INTAKE_2_TIME = 2;

    public static double LOCK_TIME = 1.2;
    public static double STOP_OUTTAKE_TIME = 1.7;

    double reflect = 1;

    public Cycles(double reflect) {
        this.reflect = reflect;
    }

    public TrajectorySequence cyclePerimeterFromFar(Robot robot, int stackHeight) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_FAR_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(225 * reflect))

                // retract turret
                .UNSTABLE_addTemporalMarkerOffset(CENTER_TURRET_TIME, () -> robot.outtake.centerTurret())
                // retract wrist
                .UNSTABLE_addTemporalMarkerOffset(WRIST_RETRACT_TIME, () -> robot.outtake.retractWrist())
                // retract lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_RETRACT_TIME, () -> robot.outtake.lift.setMotionProfileTargetPos(0))

                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-30 + Poses.FIELD_OFFSET_X, -60 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-52 + Poses.FIELD_OFFSET_X, -42 * reflect), Math.toRadians(150))

                // drop intake
                .UNSTABLE_addTemporalMarkerOffset(DROP_INTAKE_1_TIME, () -> robot.intake.setIntakeWristTargetHeight(Intake.WRIST_AUTO_READY_HEIGHT))
                // start and lower intake
                .UNSTABLE_addTemporalMarkerOffset(START_INTAKE_TIME, () -> {
                    robot.intake.dropToStack(stackHeight);
                    robot.intake.setIntakePower(1);
                })

                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .splineToSplineHeading(Poses.STACK_CLOSE_POSE, Math.toRadians(150))
                .waitSeconds(INTAKE_1_TIME)

                // lock and raise intake, start outtaking
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.lock();
                    robot.intake.setIntakeWristTargetHeight(Intake.WRIST_AUTO_READY_HEIGHT);
                    robot.intake.setIntakePower(-0.7);
                })
                // stop outtaking and retract wrist
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.intake.setIntakePower(0);
                    robot.intake.retractIntakeWrist();
                })

                .setTangent(Math.toRadians(-45 * reflect))
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .splineToConstantHeading(new Vector2d(-30 + Poses.FIELD_OFFSET_X, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.DEPOSIT_FAR_Y * reflect), Math.toRadians(0))

                // lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CYCLE_POS);
                })
                // wrist back
                .UNSTABLE_addTemporalMarkerOffset(WRIST_EXTEND_TIME, () -> {
                    robot.outtake.extendWrist();
                })
                .setVelConstraint(Constraints.NORMAL_VEL)
                .splineToConstantHeading(new Vector2d(Poses.DEPOSIT_X, Poses.DEPOSIT_FAR_Y * reflect), Math.toRadians(0))
                // release pixel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.unlock();
                })

                .waitSeconds(TOTAL_DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence cyclePerimeterFromCenter(Robot robot, int stackHeight) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_CENTER_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(225 * reflect))

                // retract turret
                .UNSTABLE_addTemporalMarkerOffset(CENTER_TURRET_TIME, () -> robot.outtake.centerTurret())
                // retract wrist
                .UNSTABLE_addTemporalMarkerOffset(WRIST_RETRACT_TIME, () -> robot.outtake.retractWrist())
                // retract lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_RETRACT_TIME, () -> robot.outtake.lift.setMotionProfileTargetPos(0))

                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-30 + Poses.FIELD_OFFSET_X, -60 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-52 + Poses.FIELD_OFFSET_X, -42 * reflect), Math.toRadians(150))

                // drop intake
                .UNSTABLE_addTemporalMarkerOffset(DROP_INTAKE_1_TIME, () -> robot.intake.setIntakeWristTargetHeight(Intake.WRIST_AUTO_READY_HEIGHT))
                // start and lower intake
                .UNSTABLE_addTemporalMarkerOffset(START_INTAKE_TIME, () -> {
                    robot.intake.dropToStack(stackHeight);
                    robot.intake.setIntakePower(1);
                })

                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .splineToSplineHeading(Poses.STACK_CLOSE_POSE, Math.toRadians(150))
                .waitSeconds(INTAKE_1_TIME)

                // lock and raise intake, start outtaking
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.lock();
                    robot.intake.setIntakeWristTargetHeight(Intake.WRIST_AUTO_READY_HEIGHT);
                    robot.intake.setIntakePower(-0.7);
                })
                // stop outtaking and retract wrist
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.intake.setIntakePower(0);
                    robot.intake.retractIntakeWrist();
                })

                .setTangent(Math.toRadians(-45 * reflect))
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .splineToConstantHeading(new Vector2d(-30 + Poses.FIELD_OFFSET_X, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.DEPOSIT_CLOSE_Y * reflect), Math.toRadians(0))

                // lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CYCLE_POS);
                })
                // wrist back
                .UNSTABLE_addTemporalMarkerOffset(WRIST_EXTEND_TIME, () -> {
                    robot.outtake.extendWrist();
                })

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.DEPOSIT_X, Poses.DEPOSIT_CLOSE_Y * reflect), Math.toRadians(0))

                // release pixel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.unlock();
                })
                .waitSeconds(TOTAL_DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence cyclePerimeterFromClose(Robot robot, int stackHeight) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_CLOSE_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(225 * reflect))

                // retract turret
                .UNSTABLE_addTemporalMarkerOffset(CENTER_TURRET_TIME, () -> robot.outtake.centerTurret())
                // retract wrist
                .UNSTABLE_addTemporalMarkerOffset(WRIST_RETRACT_TIME, () -> robot.outtake.retractWrist())
                // retract lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_RETRACT_TIME, () -> robot.outtake.lift.setMotionProfileTargetPos(0))

                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(180))
                .setVelConstraint(Constraints.NORMAL_VEL)
                .splineToConstantHeading(new Vector2d(-30 + Poses.FIELD_OFFSET_X, -60 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-52 + Poses.FIELD_OFFSET_X, -42 * reflect), Math.toRadians(150))

                // drop intake
                .UNSTABLE_addTemporalMarkerOffset(DROP_INTAKE_1_TIME, () -> robot.intake.setIntakeWristTargetHeight(Intake.WRIST_AUTO_READY_HEIGHT))
                // start and lower intake
                .UNSTABLE_addTemporalMarkerOffset(START_INTAKE_TIME, () -> {
                    robot.intake.dropToStack(stackHeight);
                    robot.intake.setIntakePower(1);
                })

                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .splineToSplineHeading(Poses.STACK_CLOSE_POSE, Math.toRadians(150))
                .waitSeconds(INTAKE_1_TIME)

                // lock and raise intake, start outtaking
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.lock();
                    robot.intake.setIntakeWristTargetHeight(Intake.WRIST_AUTO_READY_HEIGHT);
                    robot.intake.setIntakePower(-0.7);
                })
                // stop outtaking and retract wrist
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.intake.setIntakePower(0);
                    robot.intake.retractIntakeWrist();
                })

                .setTangent(Math.toRadians(-45 * reflect))
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .splineToConstantHeading(new Vector2d(-30 + Poses.FIELD_OFFSET_X, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.DEPOSIT_CLOSE_Y * reflect), Math.toRadians(0))

                // lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CYCLE_POS);
                })
                // wrist back
                .UNSTABLE_addTemporalMarkerOffset(WRIST_EXTEND_TIME, () -> {
                    robot.outtake.extendWrist();
                })

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.DEPOSIT_X, Poses.DEPOSIT_CLOSE_Y * reflect), Math.toRadians(0))

                // release pixel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.unlock();
                })
                .waitSeconds(TOTAL_DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence cycleCenterFromClose(Robot robot, int stackHeight) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_CLOSE_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(135 * reflect))
                // retract turret
                .UNSTABLE_addTemporalMarkerOffset(CENTER_TURRET_TIME, () -> robot.outtake.centerTurret())
                // retract wrist
                .UNSTABLE_addTemporalMarkerOffset(WRIST_RETRACT_TIME, () -> robot.outtake.retractWrist())
                // retract lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_RETRACT_TIME, () -> robot.outtake.lift.setMotionProfileTargetPos(0))
                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X, -12 * reflect), Math.toRadians(180))
                .setVelConstraint(Constraints.NORMAL_VEL)
                .splineToConstantHeading(new Vector2d(Poses.STACK_X - IntakeWrist.RADIUS + IntakeWrist.toX(stackHeight), -12 * reflect), Math.toRadians(180))
                // intake ready
                .UNSTABLE_addTemporalMarkerOffset(INTAKE_READY_TIME, () -> {
                    robot.intake.setIntakeWristTargetHeight(Intake.WRIST_AUTO_READY_HEIGHT);
                })
                // start intake
                .UNSTABLE_addTemporalMarkerOffset(START_INTAKE_TIME, () -> {
                    robot.intake.setIntakePower(1);
                })
                // drop intake
                .UNSTABLE_addTemporalMarkerOffset(DROP_INTAKE_1_TIME, () -> robot.intake.dropToStack(stackHeight))

                .waitSeconds(INTAKE_1_TIME)

                .UNSTABLE_addTemporalMarkerOffset(DROP_INTAKE_2_TIME, () -> {
                    robot.intake.dropToStack(0);
                })
                .lineTo(new Vector2d(Poses.STACK_SETUP_X, -12 * reflect))
                .UNSTABLE_addTemporalMarkerOffset(LOCK_TIME, () -> {
                    robot.outtake.lock();
                    robot.intake.setIntakePower(-0.7);
                    robot.intake.setIntakeWristTargetHeight(Intake.WRIST_AUTO_READY_HEIGHT);
                })
                .UNSTABLE_addTemporalMarkerOffset(STOP_OUTTAKE_TIME, () -> {
                    robot.intake.setIntakePower(0);
                    robot.intake.retractIntakeWrist();
                })
                .waitSeconds(INTAKE_2_TIME)

                .setTangent(0)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.DEPOSIT_FAR_Y * reflect), Math.toRadians(0))

                // lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CYCLE_POS);
                })
                // wrist back
                .UNSTABLE_addTemporalMarkerOffset(WRIST_EXTEND_TIME, () -> {
                    robot.outtake.extendWrist();
                })

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.DEPOSIT_X, Poses.DEPOSIT_FAR_Y * reflect), Math.toRadians(0))

                // release pixel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.unlock();
                })

                .waitSeconds(TOTAL_DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence cycleCenterFromCenter(Robot robot, int stackHeight) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_CENTER_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(135 * reflect))

                // retract turret
                .UNSTABLE_addTemporalMarkerOffset(CENTER_TURRET_TIME, () -> robot.outtake.centerTurret())
                // retract wrist
                .UNSTABLE_addTemporalMarkerOffset(WRIST_RETRACT_TIME, () -> robot.outtake.retractWrist())
                // retract lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_RETRACT_TIME, () -> robot.outtake.lift.setMotionProfileTargetPos(0))

                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X, -12 * reflect), Math.toRadians(180))
                .setVelConstraint(Constraints.NORMAL_VEL)
                .splineToConstantHeading(new Vector2d(Poses.STACK_X - IntakeWrist.RADIUS + IntakeWrist.toX(stackHeight), -12 * reflect), Math.toRadians(180))
                // intake ready
                .UNSTABLE_addTemporalMarkerOffset(INTAKE_READY_TIME, () -> {
                    robot.intake.setIntakeWristTargetHeight(Intake.WRIST_AUTO_READY_HEIGHT);
                })
                // start intake
                .UNSTABLE_addTemporalMarkerOffset(START_INTAKE_TIME, () -> {
                    robot.intake.setIntakePower(1);
                })
                // drop intake
                .UNSTABLE_addTemporalMarkerOffset(DROP_INTAKE_1_TIME, () -> robot.intake.dropToStack(stackHeight))

                .waitSeconds(INTAKE_1_TIME)

                .UNSTABLE_addTemporalMarkerOffset(DROP_INTAKE_2_TIME, () -> {
                    robot.intake.dropToStack(0);
                })
                .lineTo(new Vector2d(Poses.STACK_SETUP_X, -12 * reflect))
                .UNSTABLE_addTemporalMarkerOffset(LOCK_TIME, () -> {
                    robot.outtake.lock();
                    robot.intake.setIntakePower(-0.7);
                    robot.intake.setIntakeWristTargetHeight(Intake.WRIST_AUTO_READY_HEIGHT);
                })
                .UNSTABLE_addTemporalMarkerOffset(STOP_OUTTAKE_TIME, () -> {
                    robot.intake.setIntakePower(0);
                    robot.intake.retractIntakeWrist();
                })
                .waitSeconds(INTAKE_2_TIME)

                .setTangent(0)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.DEPOSIT_FAR_Y * reflect), Math.toRadians(0))

                // lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CYCLE_POS);
                })
                // wrist back
                .UNSTABLE_addTemporalMarkerOffset(WRIST_EXTEND_TIME, () -> {
                    robot.outtake.extendWrist();
                })

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.DEPOSIT_X, Poses.DEPOSIT_FAR_Y * reflect), Math.toRadians(0))

                // release pixel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.unlock();
                })

                .waitSeconds(TOTAL_DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence cycleCenterFromFar(Robot robot, int stackHeight) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_FAR_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(135 * reflect))

                // retract turret
                .UNSTABLE_addTemporalMarkerOffset(CENTER_TURRET_TIME, () -> robot.outtake.centerTurret())
                // retract wrist
                .UNSTABLE_addTemporalMarkerOffset(WRIST_RETRACT_TIME, () -> robot.outtake.retractWrist())
                // retract lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_RETRACT_TIME, () -> robot.outtake.lift.setMotionProfileTargetPos(0))

                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X, -12 * reflect), Math.toRadians(180))
                .setVelConstraint(Constraints.NORMAL_VEL)
                .splineToConstantHeading(new Vector2d(Poses.STACK_X - IntakeWrist.RADIUS + IntakeWrist.toX(stackHeight), -12 * reflect), Math.toRadians(180))
                // intake ready
                .UNSTABLE_addTemporalMarkerOffset(INTAKE_READY_TIME, () -> {
                    robot.intake.setIntakeWristTargetHeight(Intake.WRIST_AUTO_READY_HEIGHT);
                })
                // start intake
                .UNSTABLE_addTemporalMarkerOffset(START_INTAKE_TIME, () -> {
                    robot.intake.setIntakePower(1);
                })
                // drop intake
                .UNSTABLE_addTemporalMarkerOffset(DROP_INTAKE_1_TIME, () -> robot.intake.dropToStack(stackHeight))

                .waitSeconds(INTAKE_1_TIME)

                .UNSTABLE_addTemporalMarkerOffset(DROP_INTAKE_2_TIME, () -> {
                    robot.intake.dropToStack(0);
                })
                .lineTo(new Vector2d(Poses.STACK_SETUP_X, -12 * reflect))
                .UNSTABLE_addTemporalMarkerOffset(LOCK_TIME, () -> {
                    robot.outtake.lock();
                    robot.intake.setIntakePower(-0.7);
                    robot.intake.setIntakeWristTargetHeight(Intake.WRIST_AUTO_READY_HEIGHT);
                })
                .UNSTABLE_addTemporalMarkerOffset(STOP_OUTTAKE_TIME, () -> {
                    robot.intake.setIntakePower(0);
                    robot.intake.retractIntakeWrist();
                })
                .waitSeconds(INTAKE_2_TIME)
                .setTangent(0)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.DEPOSIT_FAR_Y * reflect), Math.toRadians(0))

                // lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CYCLE_POS);
                })
                // wrist back
                .UNSTABLE_addTemporalMarkerOffset(WRIST_EXTEND_TIME, () -> {
                    robot.outtake.extendWrist();
                })
                .UNSTABLE_addTemporalMarkerOffset(TURN_TURRET_TIME, () -> {
                    robot.outtake.setTurretAngle(270 + 40 * reflect);
                })


                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.DEPOSIT_X, Poses.DEPOSIT_FAR_Y * reflect), Math.toRadians(0))

                // release pixel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.unlock();
                })

                .waitSeconds(TOTAL_DEPOSIT_TIME)
                .build();
    }
}

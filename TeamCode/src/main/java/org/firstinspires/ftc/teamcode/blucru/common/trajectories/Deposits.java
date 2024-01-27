package org.firstinspires.ftc.teamcode.blucru.common.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class Deposits {
    public static double DEPOSIT_TIME = 1.2;
    public static double INTAKE_TIME = 1.5;

    public static double reflect = 1;

    public Deposits(double reflect) {
        this.reflect = reflect;
    }

    public TrajectorySequence cyclePerimeterFromFar(Robot robot, int stackHeight) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_FAR_POSE)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(Math.toRadians(225 * reflect))
                // retract turret

                // retract wrist

                // retract lift

                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-30, -60 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X, -36 * reflect), Math.toRadians(180))
                // start and lower intake

                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -36 * reflect), Math.toRadians(180))
                .waitSeconds(0.5)
                // lock and raise intake, start outtaking

                // stop outtaking

                .setTangent(Math.toRadians(-45 * reflect))
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .splineToConstantHeading(new Vector2d(-30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_FAR_Y * reflect), Math.toRadians(0))
                // lift

                // wrist back

                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_X, Poses.BACKDROP_FAR_Y * reflect), Math.toRadians(0))
                // release pixel

                .waitSeconds(DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence cyclePerimeterFromCenter(Robot robot, int stackHeight) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_CENTER_POSE)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(Math.toRadians(225 * reflect))
                // retract turret

                // retract wrist

                // retract lift

                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-30, -60 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X, -36 * reflect), Math.toRadians(180))
                // start and lower intake

                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -36 * reflect), Math.toRadians(180))
                .waitSeconds(0.5)
                // lock and raise intake, start outtaking

                // stop outtaking

                .setTangent(Math.toRadians(-45 * reflect))
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .splineToConstantHeading(new Vector2d(-30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_CLOSE_Y * reflect), Math.toRadians(0))
                // lift

                // wrist back

                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_X, Poses.BACKDROP_CLOSE_Y * reflect), Math.toRadians(0))
                // release pixel

                .waitSeconds(DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence cyclePerimeterFromClose(Robot robot, int stackHeight) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_CLOSE_POSE)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(Math.toRadians(225 * reflect))
                // retract turret

                // retract wrist

                // retract lift

                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-30, -60 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X, -36 * reflect), Math.toRadians(180))
                // start and lower intake

                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -36 * reflect), Math.toRadians(180))
                .waitSeconds(0.5)
                // lock and raise intake, start outtaking

                // stop outtaking

                .setTangent(Math.toRadians(-45 * reflect))
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .splineToConstantHeading(new Vector2d(-30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_CLOSE_Y * reflect), Math.toRadians(0))
                // lift

                // wrist back

                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_X, Poses.BACKDROP_CLOSE_Y * reflect), Math.toRadians(0))
                // release pixel

                .waitSeconds(DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence cycleCenterFromClose(Robot robot, int stackHeight) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_CLOSE_POSE)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(Math.toRadians(135 * reflect))
                // retract turret

                // retract wrist

                // retract lift

                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X, -12 * reflect), Math.toRadians(180))
                // start and lower intake

                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -12 * reflect), Math.toRadians(180))
                .waitSeconds(0.5)
                // lock and raise intake, start outtaking

                // stop outtaking

                .setTangent(0)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_FAR_Y * reflect), Math.toRadians(0))
                // lift

                // wrist back

                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_X, Poses.BACKDROP_FAR_Y * reflect), Math.toRadians(0))
                // release pixel

                .waitSeconds(DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence cycleCenterFromCenter(Robot robot, int stackHeight) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_CENTER_POSE)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(Math.toRadians(135 * reflect))
                // retract turret

                // retract wrist

                // retract lift

                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X, -12 * reflect), Math.toRadians(180))
                // start and lower intake

                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -12 * reflect), Math.toRadians(180))
                .waitSeconds(0.5)
                // lock and raise intake, start outtaking

                // stop outtaking

                .setTangent(0)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_FAR_Y * reflect), Math.toRadians(0))
                // lift

                // wrist back

                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_X, Poses.BACKDROP_FAR_Y * reflect), Math.toRadians(0))
                // release pixel

                .waitSeconds(DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence cycleCenterFromFar(Robot robot, int stackHeight) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_FAR_POSE)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(Math.toRadians(135 * reflect))
                // retract turret

                // retract wrist

                // retract lift

                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X, -12 * reflect), Math.toRadians(180))
                // start and lower intake

                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -12 * reflect), Math.toRadians(180))
                .waitSeconds(0.5)
                // lock and raise intake, start outtaking

                // stop outtaking

                .setTangent(0)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_FAR_Y * reflect), Math.toRadians(0))
                // lift

                // wrist back

                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_X, Poses.BACKDROP_FAR_Y * reflect), Math.toRadians(0))
                // release pixel

                .waitSeconds(DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence depositFromBackdropClose(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.BACKDROP_PLACEMENT_CLOSE_POSE)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(Math.toRadians(270 * reflect))
                .splineToConstantHeading(new Vector2d(30, -44 * reflect), 0)
                .splineToSplineHeading(new Pose2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_CLOSE_Y * reflect, Math.toRadians(180)), 0)
                // lift

                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                .splineToConstantHeading(Poses.DEPOSIT_CLOSE_POSE.vec(), 0)
                // deposit

                .waitSeconds(DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence depositFromBackdropCenter(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.BACKDROP_PLACEMENT_CENTER_POSE)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(Math.toRadians(-90 * reflect))
                .splineToConstantHeading(new Vector2d(20, -36 * reflect), 0)
                .splineToSplineHeading(new Pose2d(Poses.BACKDROP_SETUP_X, -36*reflect, Math.toRadians(180)), 0)
                // lift

                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                .splineToConstantHeading(Poses.DEPOSIT_CENTER_POSE.vec(), Math.toRadians(0))
                // release

                .waitSeconds(DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence depositFromBackdropFar(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.BACKDROP_PLACEMENT_FAR_POSE)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(-45*reflect)
                .splineToConstantHeading(new Vector2d(8, -42 * reflect), 0)
                .splineToConstantHeading(new Vector2d(10, -42*reflect), 0)
                .splineToSplineHeading(new Pose2d(30, -35 * reflect, Math.toRadians(180)), Math.toRadians(45*reflect))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_FAR_Y*reflect), 0)
                // lift

                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                .splineToConstantHeading(Poses.DEPOSIT_FAR_POSE.vec(), Math.toRadians(0))
                // deposit

                .waitSeconds(DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence depositThroughCenterFromWingClose(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.WING_PLACEMENT_CLOSE_FOR_CENTER_POSE)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(Math.toRadians(180 * reflect))
                .splineToConstantHeading(new Vector2d(-53, -24*reflect), Math.toRadians(180))
                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                // drop down, start intake, unlock

                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -24 * reflect), Math.toRadians(180))
                .waitSeconds(INTAKE_TIME)
                // lock and stop intake

                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(-40, -12 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, -36 * reflect), Math.toRadians(0))
                // lift

                // wrist back

                // turn turret

                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                .splineToConstantHeading(Poses.DEPOSIT_CENTER_POSE.vec(), Math.toRadians(0))
                // release white pixel

                // turn turret

                // release yellow pixel

                .waitSeconds(DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence depositThroughCenterFromWingCenter(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.WING_PLACEMENT_CENTER_POSE)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(Math.toRadians(180 * reflect))
                .splineToConstantHeading(new Vector2d(-53, -24*reflect), Math.toRadians(180))
                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                // drop down, start intake, unlock

                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -24 * reflect), Math.toRadians(180))
                .waitSeconds(INTAKE_TIME)
                // lock and start outtaking

                // stop outtake

                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(Math.toRadians(45 * reflect))
                .splineToConstantHeading(new Vector2d(-45, -12 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, -36 * reflect), Math.toRadians(0))
                // lift

                // wrist back

                // turn turret

                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                .splineToConstantHeading(Poses.DEPOSIT_CENTER_POSE.vec(), Math.toRadians(0))
                // release white pixel

                // turn turret

                // release yellow pixel

                .waitSeconds(DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence depositThroughCenterFromWingFar(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.WING_PLACEMENT_FAR_FOR_PERIM_POSE)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(Math.toRadians(180 * reflect))
                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                // drop down, start intake, unlock

                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -24 * reflect), Math.toRadians(180))
                .waitSeconds(INTAKE_TIME)
                // lock and start outtaking

                // stop outtake

                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(Math.toRadians(45 * reflect))
                .splineToConstantHeading(new Vector2d(-45, -12 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_FAR_Y * reflect), Math.toRadians(0))
                // lift

                // wrist back

                // turn turret

                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                .splineToConstantHeading(Poses.DEPOSIT_FAR_POSE.vec(), Math.toRadians(0))
                // release white pixel

                // turn turret

                // release yellow pixel

                .waitSeconds(DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence depositThroughPerimeterFromWingClose(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.WING_PLACEMENT_CLOSE_FOR_PERIM_POSE)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(Math.toRadians(180 * reflect))
                .splineToSplineHeading(new Pose2d(Poses.STACK_SETUP_X, -36*reflect, Math.toRadians(180)), Math.toRadians(180))
                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                // drop down, start intake, unlock

                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -36 * reflect), Math.toRadians(180))
                .waitSeconds(INTAKE_TIME)
                // lock and start outtaking

                // stop outtake

                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(Math.toRadians(-45 * reflect))
                .splineToConstantHeading(new Vector2d(-45, -60 * reflect), Math.toRadians(0))
                .setConstraints(Constraints.NORMAL_VELOCITY, Constraints.NORMAL_ACCELERATION)
                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_CLOSE_Y * reflect), Math.toRadians(0))
                // lift

                // wrist back

                // turn turret

                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                .splineToConstantHeading(Poses.DEPOSIT_CLOSE_POSE.vec(), Math.toRadians(0))
                // release white pixel

                // turn turret

                // release yellow pixel

                .waitSeconds(DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence depositThroughPerimeterFromWingCenter(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.WING_PLACEMENT_CENTER_POSE)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(Math.toRadians(225 * reflect))
                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X, -24 * reflect), Math.toRadians(180))
                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                // drop down, start intake, unlock

                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -24 * reflect), Math.toRadians(180))
                .waitSeconds(INTAKE_TIME)
                // lock and start outtaking

                // stop outtake

                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(Math.toRadians(-45 * reflect))
                .splineToConstantHeading(new Vector2d(-45, -60 * reflect), Math.toRadians(0))
                .setConstraints(Constraints.NORMAL_VELOCITY, Constraints.NORMAL_ACCELERATION)
                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_CLOSE_Y * reflect), Math.toRadians(0))
                // lift

                // wrist back

                // turn turret

                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                .splineToConstantHeading(Poses.DEPOSIT_CLOSE_POSE.vec(), Math.toRadians(0))
                // release white pixel

                // turn turret

                // release yellow pixel

                .waitSeconds(DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence depositThroughPerimeterFromWingFar(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.WING_PLACEMENT_FAR_FOR_PERIM_POSE)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(Math.toRadians(180 * reflect))
                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                // drop down, start intake, unlock

                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -36 * reflect), Math.toRadians(180))
                .waitSeconds(INTAKE_TIME)
                // lock and start outtaking

                // stop outtake

                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(Math.toRadians(-45 * reflect))
                .splineToConstantHeading(new Vector2d(-45, -60 * reflect), Math.toRadians(0))
                .setConstraints(Constraints.NORMAL_VELOCITY, Constraints.NORMAL_ACCELERATION)
                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, -36 * reflect), Math.toRadians(0))
                // lift

                // wrist back

                // turn turret

                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                .splineToConstantHeading(Poses.DEPOSIT_CENTER_POSE.vec(), Math.toRadians(0))
                // release white pixel

                // turn turret

                // release yellow pixel

                .waitSeconds(DEPOSIT_TIME)
                .build();
    }
}

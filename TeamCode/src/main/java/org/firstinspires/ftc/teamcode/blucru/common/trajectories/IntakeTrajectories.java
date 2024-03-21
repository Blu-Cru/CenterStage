package org.firstinspires.ftc.teamcode.blucru.common.trajectories;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class IntakeTrajectories {
    public static double
            DROP_INTAKE_TIME = -1.0,
            START_INTAKE_TIME = -0.5,
            INTAKE_LENGTH = 0.0,
            INTAKE_HYPOTENUSE = INTAKE_LENGTH * INTAKE_LENGTH;

    double reflect;
    public IntakeTrajectories(double reflect) {
        reflect = reflect;
    }

    public TrajectorySequence intakeCenterFromFar(Robot robot, int stackHeight) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_CLOSE_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(135 * reflect))
                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X, -12 * reflect), Math.toRadians(180))
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .splineToConstantHeading(new Vector2d(-54, -8 * reflect), Math.toRadians(180 * reflect))
                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -16 * reflect), Math.toRadians(-90 * reflect))
                .splineToConstantHeading(new Vector2d(-60, -8 * reflect), Math.toRadians(90 * reflect))
                .build();
    }
}

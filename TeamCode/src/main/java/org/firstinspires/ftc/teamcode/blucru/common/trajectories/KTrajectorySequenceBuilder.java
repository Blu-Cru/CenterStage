package org.firstinspires.ftc.teamcode.blucru.common.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class KTrajectorySequenceBuilder extends TrajectorySequenceBuilder {
    public static double reflect;

    public KTrajectorySequenceBuilder(Pose2d startPose,
                                      TrajectoryVelocityConstraint velocityConstraint,
                                      TrajectoryAccelerationConstraint accelerationConstraint,
                                      double maxAngVel,
                                      double maxAngAccel) {
        super(startPose,
                velocityConstraint,
              accelerationConstraint,
              maxAngVel,
              maxAngAccel);
    }

    public static void setReflect(double reflect) {
        KTrajectorySequenceBuilder.reflect = reflect;
    }

    public KTrajectorySequenceBuilder splineToConstantHeading(double x, double y, double endTangent) {
        super.splineToConstantHeading(new Vector2d(x, y * reflect), Math.toRadians(endTangent * reflect));
        return this;
    }

    public KTrajectorySequenceBuilder splineToConstantHeading(Vector2d endPos, double endTangent) {
        super.splineToConstantHeading(new Vector2d(endPos.getX(), endPos.getY() * reflect), Math.toRadians(endTangent * reflect));
        return this;
    }

    public KTrajectorySequenceBuilder splineToSplineHeading(double x, double y, double heading, double endTangent) {
        super.splineToSplineHeading(new Pose2d(x, y * reflect, Math.toRadians(heading * reflect)), Math.toRadians(endTangent * reflect));
        return this;
    }

    public KTrajectorySequenceBuilder splineToSplineHeading(Pose2d endPose, double endTangent) {
        super.splineToSplineHeading(new Pose2d(endPose.getX(), endPose.getY() * reflect, Math.toRadians(endPose.getHeading() * reflect)), Math.toRadians(endTangent * reflect));
        return this;
    }

    public KTrajectorySequenceBuilder setConstraints(int constraint) {
        super.setConstraints(Constraints.velos[constraint], Constraints.accels[constraint]);
        return this;
    }

    public KTrajectorySequenceBuilder setTangent(double degrees) {
        super.setTangent(Math.toRadians(degrees * reflect));
        return this;
    }

    public KTrajectorySequenceBuilder waitSeconds(double seconds) {
        super.waitSeconds(seconds);
        return this;
    }

    public KTrajectorySequenceBuilder addTemporalMarkerOffset(double seconds, MarkerCallback callback) {
        super.UNSTABLE_addTemporalMarkerOffset(seconds, callback);
        return this;
    }

    public KTrajectorySequenceBuilder intakeCenter(int stackHeight) {
        this.setConstraints(1)
        .setTangent(135)
        // retract turret

        // retract wrist

        // retract lift

        .waitSeconds(1);
        return this;
    }

    public TrajectorySequence build() {
        return super.build();
    }
}

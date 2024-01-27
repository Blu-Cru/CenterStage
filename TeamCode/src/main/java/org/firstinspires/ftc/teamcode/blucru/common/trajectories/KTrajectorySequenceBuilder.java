package org.firstinspires.ftc.teamcode.blucru.common.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

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
}

//package org.firstinspires.ftc.teamcode.blucru.common.trajectories;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
//
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
//
//// custom trajectory sequence builder for BluCru
//public class BCTrajectorySequenceBuilder extends TrajectorySequenceBuilder {
//    public static double reflect;
//
//    public BCTrajectorySequenceBuilder(Pose2d startPose,
//                                       TrajectoryVelocityConstraint velocityConstraint,
//                                       TrajectoryAccelerationConstraint accelerationConstraint,
//                                       double maxAngVel,
//                                       double maxAngAccel) {
//        super(startPose,
//                velocityConstraint,
//              accelerationConstraint,
//              maxAngVel,
//              maxAngAccel);
//    }
//
//    public static void setReflect(double reflect) {
//        BCTrajectorySequenceBuilder.reflect = reflect;
//    }
//
//    public BCTrajectorySequenceBuilder splineToConstantHeading(double x, double y, double endTangent) {
//        super.splineToConstantHeading(new Vector2d(x, y * reflect), Math.toRadians(endTangent * reflect));
//        return this;
//    }
//
//    public BCTrajectorySequenceBuilder splineToConstantHeading(Vector2d endPos, double endTangent) {
//        super.splineToConstantHeading(new Vector2d(endPos.getX(), endPos.getY() * reflect), Math.toRadians(endTangent * reflect));
//        return this;
//    }
//
//    public BCTrajectorySequenceBuilder splineToSplineHeading(double x, double y, double heading, double endTangent) {
//        super.splineToSplineHeading(new Pose2d(x, y * reflect, Math.toRadians(heading * reflect)), Math.toRadians(endTangent * reflect));
//        return this;
//    }
//
//    public BCTrajectorySequenceBuilder splineToSplineHeading(Pose2d endPose, double endTangent) {
//        super.splineToSplineHeading(new Pose2d(endPose.getX(), endPose.getY() * reflect, Math.toRadians(endPose.getHeading() * reflect)), Math.toRadians(endTangent * reflect));
//        return this;
//    }
//
//    public BCTrajectorySequenceBuilder setConstraints(int constraint) {
//        super.setConstraints(Constraints.velos[constraint], Constraints.accels[constraint]);
//        return this;
//    }
//
//    public BCTrajectorySequenceBuilder setTangent(double degrees) {
//        super.setTangent(Math.toRadians(degrees * reflect));
//        return this;
//    }
//
//    public BCTrajectorySequenceBuilder waitSeconds(double seconds) {
//        super.waitSeconds(seconds);
//        return this;
//    }
//
//    public BCTrajectorySequenceBuilder addTemporalMarkerOffset(double seconds, MarkerCallback callback) {
//        super.UNSTABLE_addTemporalMarkerOffset(seconds, callback);
//        return this;
//    }
//
//    public BCTrajectorySequenceBuilder intakeCenter(int stackHeight) {
//        this.setConstraints(1)
//        .setTangent(135)
//        // retract turret
//
//        // retract wrist
//
//        // retract lift
//
//        .waitSeconds(1);
//        return this;
//    }
//
//    public TrajectorySequence build() {
//        return super.build();
//    }
//}

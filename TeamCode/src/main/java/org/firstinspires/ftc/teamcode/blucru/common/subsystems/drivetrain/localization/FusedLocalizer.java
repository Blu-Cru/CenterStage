package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.teamcode.blucru.common.vision.CVMaster;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class FusedLocalizer implements Localizer {
    StandardTrackingWheelLocalizer deadWheels;
    PoseHistory poseHistory;

    Pose2d poseEstimate;

    @NonNull
    public Pose2d getPoseEstimate() {
        return deadWheels.getPoseEstimate();
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        deadWheels.setPoseEstimate(pose2d);
        poseEstimate = pose2d;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return deadWheels.getPoseVelocity();
    }

    @Override
    public void update() {
        deadWheels.update();
        poseHistory.add(deadWheels.getPoseEstimate());
    }

    public void update(AprilTagProcessor tagProcessor) {
        ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
        if(detections.size() < 1) return; // dont update if there are no detections

        Pose2d tagPoseTimeOfFrame = AprilTagPoseGetter.getRobotPoseAtTimeOfFrame(detections);
        Pose2d odoPoseTimeOfFrame = poseHistory.getPoseAtTime(detections.get(0).frameAcquisitionNanoTime);


    }
}

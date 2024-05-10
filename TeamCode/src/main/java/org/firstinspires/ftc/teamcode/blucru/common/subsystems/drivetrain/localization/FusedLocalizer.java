package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization;

import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class FusedLocalizer {
    Localizer deadWheels;
    PoseHistory poseHistory;

    public FusedLocalizer(Localizer localizer) {
        deadWheels = localizer;
        poseHistory = new PoseHistory();
    }

    public void update() {
        // make a copy of the current pose, so that the pose history doesn't get updated with the same object
        Pose2d currentPose = deadWheels.getPoseEstimate();
        poseHistory.add(new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getHeading()));
    }

    public void updateAprilTags(AprilTagProcessor tagProcessor) {
        ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
        if(detections.size() < 1) return;

        // save reference to tag pose
        Pose2d tagPoseTimeOfFrame = AprilTagPoseGetter.getRobotPoseAtTimeOfFrame(detections);
        // get odo pose at the time of the tag pose
        Pose2d odoPoseTimeOfFrame = poseHistory.getPoseAtTime(detections.get(0).frameAcquisitionNanoTime);

        // calculate change from old odo pose to current pose
        Pose2d odoDeltaPoseFrameToNow = deadWheels.getPoseEstimate().minus(odoPoseTimeOfFrame);
        Log.v("Fused Localizer", "Odo delta: x " + odoDeltaPoseFrameToNow.getX() + " y " + odoDeltaPoseFrameToNow.getY());

        // set pose estimate to tag pose + delta
        deadWheels.setPoseEstimate(tagPoseTimeOfFrame.plus(odoDeltaPoseFrameToNow));
        // add tag - odo to pose history
        Pose2d odoPoseError = tagPoseTimeOfFrame.minus(odoPoseTimeOfFrame);
        poseHistory.offset(odoPoseError);
    }

    public double getWeight() {
        return 1.0;
    }
}

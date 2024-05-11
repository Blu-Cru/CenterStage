package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization;

import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class FusedLocalizer {
    Localizer deadWheels;
    IMU imu;
    PoseHistory poseHistory;

    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

    double headingOffset = 0;
    double lastImuUpdateMillis = 0;
    public boolean usingIMU = true;

    YawPitchRollAngles ypr;

    public FusedLocalizer(Localizer localizer, HardwareMap hardwareMap) {
        deadWheels = localizer;
        poseHistory = new PoseHistory();

        imu = hardwareMap.get(IMU.class, "e hub imu");
    }

    public void update() {
        // make a copy of the current pose, so that the pose history doesn't get updated with the same object
        deadWheels.update();
        Pose2d currentPose = deadWheels.getPoseEstimate();
        poseHistory.add(new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getHeading()));

        if(System.currentTimeMillis() - lastImuUpdateMillis > 100) {
            lastImuUpdateMillis = System.currentTimeMillis();

            ypr = imu.getRobotYawPitchRollAngles();
            Log.v("FusedLocalizer", "Updating IMU, correction = " + (ypr.getYaw(AngleUnit.RADIANS) + headingOffset - deadWheels.getPoseEstimate().getHeading()));
            Pose2d currentPoseWithHeading = new Pose2d(currentPose.getX(), currentPose.getY(), ypr.getYaw(AngleUnit.RADIANS) + headingOffset);
            deadWheels.setPoseEstimate(currentPoseWithHeading);
        }
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

        // set pose estimate to tag pose + delta
        deadWheels.setPoseEstimate(tagPoseTimeOfFrame.plus(odoDeltaPoseFrameToNow));
        // add tag - odo to pose history
        Pose2d odoPoseError = tagPoseTimeOfFrame.minus(odoPoseTimeOfFrame);
        poseHistory.offset(odoPoseError);
    }

    public void init() {
        imu.resetDeviceConfigurationForOpMode();
        imu.initialize(parameters);

        lastImuUpdateMillis = System.currentTimeMillis();
    }

    public void resetHeading(double newHeading) {
        headingOffset = newHeading - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        deadWheels.setPoseEstimate(new Pose2d(deadWheels.getPoseEstimate().vec(), newHeading));
    }

    public double getWeight() {
        return 1.0;
    }
}

package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

import java.util.ArrayList;

public class PoseHistory {
    static double STORAGE_NANOSECONDS = 2.0 * Math.pow(10.0, 9.0);

    static class PoseMarker {
        double nanoTime;
        Pose2d pose;

        PoseMarker() {
            nanoTime = System.nanoTime();
            pose = Robot.getInstance().drivetrain.pose;
        }
    }

    ArrayList<PoseMarker> poseHistory;

    public PoseHistory() {
        poseHistory = new ArrayList<>();
    }

    public void addPose() {
        poseHistory.add(new PoseMarker());

        double currentTime = System.nanoTime();
        while (poseHistory.size() > 0 && currentTime - poseHistory.get(0).nanoTime > STORAGE_NANOSECONDS) {
            poseHistory.remove(0);
        }
    }
}

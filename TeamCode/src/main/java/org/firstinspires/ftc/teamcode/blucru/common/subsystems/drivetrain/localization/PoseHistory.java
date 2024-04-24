package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

import java.util.ArrayList;

public class PoseHistory {
    static double STORAGE_NANOSECONDS = 1.0 * Math.pow(10.0, 9.0);

    static class PoseMarker {
        double nanoTime;
        Pose2d pose;

        PoseMarker() {
            nanoTime = System.nanoTime();
            pose = Robot.getInstance().drivetrain.pose;
        }
    }

    ArrayList<PoseMarker> poseHistory;

    /*
        list of poses and their timestamps
        poses are stored in the order they were added, so new poses will be at the end of the list
        timestamps are in nanoseconds
     */

    public PoseHistory() {
        // initialize pose history
        poseHistory = new ArrayList<>();
    }

    public void addPose() {
        poseHistory.add(new PoseMarker());

        double currentTime = System.nanoTime();

        // remove old poses
        while (poseHistory.size() > 0 && currentTime - poseHistory.get(0).nanoTime > STORAGE_NANOSECONDS) {
            poseHistory.remove(0);
        }
    }

    public Pose2d getPoseAtTime(double targetNanoTime) {
        for (int i = poseHistory.size() - 1; i >= 0; i-= 2) { // increment by 2 for more efficiency
            if (poseHistory.get(i).nanoTime < targetNanoTime) {
                return poseHistory.get(i + 1).pose; // return the pose after the pose marker at the target time because incrementing by 2
            }
        }
        Log.e("PoseHistory", "No pose found at time " + targetNanoTime);
        return null;
    }
}

package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.ListIterator;

public class PoseHistory {
    static double STORAGE_NANOSECONDS = 1.0 * Math.pow(10.0, 9.0);

    static class PoseMarker {
        long nanoTime;
        Pose2d pose;

        PoseMarker(Pose2d pose) {
            nanoTime = System.nanoTime();
            this.pose = pose;
        }
    }

    LinkedList<PoseMarker> poseHistory;

    /*
        Linked list of poses and their timestamps
        New poses are stored at the front of the linked list
        Timestamps are in nanoseconds

        The reason for using a linked list is because we need
        to add and remove elements from the front
        and back of the list, which is faster with a
        linked list than an arraylist
     */

    public PoseHistory() {
        // initialize pose history
        poseHistory = new LinkedList<>();
    }

    public void add(Pose2d pose) {
        poseHistory.addFirst(new PoseMarker(pose));

        long currentTime = System.nanoTime();

        // remove old poses
        while (poseHistory.size() > 0 && currentTime - poseHistory.getLast().nanoTime > STORAGE_NANOSECONDS) {
            poseHistory.removeLast();
        }
    }

    public Pose2d getPoseAtTime(long targetNanoTime) {
        ListIterator<PoseMarker> iterator = poseHistory.listIterator();
        PoseMarker poseMarker = iterator.next();

        while(iterator.hasNext()) {
            if (poseMarker.nanoTime < targetNanoTime) {
                return poseMarker.pose;
            }
            poseMarker = iterator.next();
        }

        Log.e("PoseHistory", "No pose found at time " + targetNanoTime);
        return null;
    }
}

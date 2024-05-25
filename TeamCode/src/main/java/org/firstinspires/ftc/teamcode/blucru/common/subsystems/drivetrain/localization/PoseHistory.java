package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.LinkedList;
import java.util.ListIterator;
import java.util.Queue;

public class PoseHistory {
    static double STORAGE_NANOSECONDS = 1.0 * Math.pow(10.0, 9.0); // 1 second

    LinkedList<PoseMarker> poseList;

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
        // TODO: change to queue, remove until pose is found
        // initialize pose history
        poseList = new LinkedList<>();
    }

    public void add(Pose2d pose) {
        poseList.addFirst(new PoseMarker(pose)); // add current pose to front of list

        // remove old poses
        long currentTime = System.nanoTime();
        while (poseList.size() > 0 && currentTime - poseList.getLast().nanoTime > STORAGE_NANOSECONDS) {
            poseList.removeLast(); // remove oldest pose from back of list until we have less than STORAGE_NANOSECONDS of poses
        }
    }

    public Pose2d getPoseAtTime(long targetNanoTime) {
//        ListIterator<PoseMarker> iterator = poseList.listIterator();
//
//        while(iterator.hasNext()) {
//            PoseMarker poseMarker = iterator.next();
//            if (poseMarker.nanoTime < targetNanoTime) {
////                Log.i("PoseHistory", "found: " + poseMarker.pose);
//                return poseMarker.pose;
//            }
////            Log.v("PoseHistory", "iterated: " + poseMarker.pose + ", pose hash code = " + poseMarker.pose.hashCode());
//        }
        for(PoseMarker poseMarker : poseList) {
            if (poseMarker.nanoTime < targetNanoTime) {
//                Log.i("PoseHistory", "found: " + poseMarker.pose);
                return poseMarker.pose;
            }
//            Log.v("PoseHistory", "iterated: " + poseMarker.pose + ", pose hash code = " + poseMarker.pose.hashCode());
        }

//        Log.e("PoseHistory", "No pose found at time " + targetNanoTime);
        return null;
    }

    public void offset(Pose2d poseDelta) {
        for (PoseMarker marker : poseList) {
            marker.pose = new Pose2d(marker.pose.vec().plus(poseDelta.vec()), marker.pose.getHeading());
        }
    }
}
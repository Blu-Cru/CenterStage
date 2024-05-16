package org.firstinspires.ftc.teamcode.blucru.common.states;

import com.acmerobotics.roadrunner.geometry.Pose2d;

// class for the initialization state to start teleop
public class Globals {

    // default pose for the robot, will be changed at the end of auto
    public static Pose2d START_POSE = new Pose2d(0, 0, Math.toRadians(90));

    // default alliance is red, but will be changed before auto starts
    public static Alliance ALLIANCE = Alliance.RED;
}

package org.firstinspires.ftc.teamcode.blucru.common.states;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// class for the initialization state to start teleop
public class Globals {

    // default pose for the robot, will be changed at the end of auto
    public static Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

    // default alliance is red, but will be changed before auto starts
    public static Alliance alliance = Alliance.RED;
    public static Side side = Side.CLOSE;
    public static AutoType autoType = AutoType.CENTER_CYCLE;
    public static ParkType parkType = ParkType.CENTER;

    public static ElapsedTime runtime;

    public static void startTimer() {
        runtime = new ElapsedTime();
        runtime.reset();
    }

    public static void autoConfigTelemetry(Telemetry telemetry) {
        telemetry.addData("□ to cycle ALLIANCE:", alliance);
        telemetry.addData("△ to cycle SIDE:", side);
        telemetry.addData("⨉ to cycle AUTO TYPE:", autoType);
        telemetry.addData("◯ to cycle PARK:", parkType);
    }

    public static void setStartPose(Pose2d pose) {
        startPose = alliance == Alliance.RED ? pose : new Pose2d(pose.getX(), -pose.getY(), pose.getHeading() + Math.PI);
    }
}

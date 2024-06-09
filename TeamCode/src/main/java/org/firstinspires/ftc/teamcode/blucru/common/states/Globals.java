package org.firstinspires.ftc.teamcode.blucru.common.states;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Poses;

public class Globals {
    // default pose for the robot, will be changed at the end of auto
    public static Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

    public static double reflect = -1;

    // default alliance is red, but will be changed before auto starts
    public static Alliance alliance = Alliance.RED;
    public static Side side = Side.AUDIENCE;
    public static AutoType autoType = AutoType.PRELOAD;
    public static ParkType parkType = ParkType.NONE;

    public static ElapsedTime runtime;

    public static double voltage = 13.0;

    public static void startTimer() {
        runtime = new ElapsedTime();
        runtime.reset();
    }

    public static void setStartPose(Pose2d pose) {
        startPose = mapPose(pose.getX(), pose.getY(), pose.getHeading());
    }

    public static void setAutoStartPose() {
        if(side == Side.AUDIENCE) setStartPose(new Pose2d(-36, 62, Math.toRadians(90)));
        else setStartPose(new Pose2d(12, 62, Math.toRadians(90)));
    }

    public static void setAlliance(Alliance alliance) {
        Globals.alliance = alliance;
        Globals.reflect = alliance == Alliance.RED ? -1 : 1;
        Poses.setAlliance(alliance);
    }

    public static Pose2d mapPose(double x, double y, double headingDegrees) {
        return new Pose2d(x, y * reflect, Math.toRadians(headingDegrees * reflect));
    }

    public static void setVoltage(double voltage) {
        Globals.voltage = voltage;
    }

    public static double correctPower(double power) {
        return power * 13.0 / Globals.voltage;
    }

    public static void autoConfigTelemetry(Telemetry telemetry) {
        telemetry.addData("□ to cycle ALLIANCE:", alliance);
        telemetry.addData("△ to cycle SIDE:", side);
        telemetry.addData("⨉ to cycle AUTO TYPE:", autoType);
        telemetry.addData("◯ to cycle PARK:", parkType);
        telemetry.addLine("Right stick button to build");
    }

    public static void autoConfigStatus(Telemetry telemetry) {
        telemetry.addData("Alliance:", alliance);
        telemetry.addData("Side:", side);
        telemetry.addData("Auto Type:", autoType);
        telemetry.addData("Park Type:", parkType);
    }
}

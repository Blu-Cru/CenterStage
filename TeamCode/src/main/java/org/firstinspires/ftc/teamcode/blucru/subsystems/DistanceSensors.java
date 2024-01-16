package org.firstinspires.ftc.teamcode.blucru.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensors implements Subsystem {
    public static double DISTANCE_SENSOR_OFFSET = 6.5; // distance between distance sensors in inches
    public static double DISTANCE_SENSOR_MAX = 60; // max distance the distance sensors can read
    DistanceSensor rightDistanceSensor;
    DistanceSensor leftDistanceSensor;

    double rightDistance;
    double leftDistance;
    public double angle; // angle of the robot relative to the wall, 0 being dead on, 90 being parallel turned counter clockwise
    private double averageDistance;
    public double distanceFromWall; // distance from the wall, 0 being touching the wall

    public DistanceSensors(HardwareMap hardwareMap) {
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "right distance");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "left distance");
    }
    @Override
    public void init() {
        rightDistance = 0;
        leftDistance = 0;
        angle = 0;
        averageDistance = 0;
        distanceFromWall = 0;
    }

    public void read() {
        double rawRightDistance = rightDistanceSensor.getDistance(DistanceUnit.INCH);
        double rawLeftDistance = leftDistanceSensor.getDistance(DistanceUnit.INCH);

        if(rawRightDistance < DISTANCE_SENSOR_MAX && rawLeftDistance < DISTANCE_SENSOR_MAX) {
            rightDistance = rawRightDistance;
            leftDistance = rawLeftDistance;
            angle = Math.toDegrees(Math.atan((rightDistance - leftDistance) / DISTANCE_SENSOR_OFFSET));
            averageDistance = (rightDistance + leftDistance) / 2;
            distanceFromWall = averageDistance * Math.cos(Math.toRadians(angle));
        }
    }

    @Override
    public void write() {

    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("angle", angle);
        telemetry.addData("average distance", averageDistance);
        telemetry.addData("distance from wall", distanceFromWall);
    }

    public void testTelemetry(Telemetry telemetry) {
        telemetry.addData("right distance", rightDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("left distance", leftDistanceSensor.getDistance(DistanceUnit.INCH));
    }
}

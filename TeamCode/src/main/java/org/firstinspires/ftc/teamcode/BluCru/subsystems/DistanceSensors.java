package org.firstinspires.ftc.teamcode.BluCru.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensors implements Subsystem {
    public static double DISTANCE_SENSOR_OFFSET = 6.5; // distance between distance sensors in inches
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

    }

    @Override
    public void update() {
        double rawRightDistance = rightDistanceSensor.getDistance(DistanceUnit.INCH);
        double rawLeftDistance = leftDistanceSensor.getDistance(DistanceUnit.INCH);

        if(rawRightDistance < 60) {
            rightDistance = rawRightDistance;
        }

        if(rawLeftDistance < 60) {
            leftDistance = rawLeftDistance;
        }
        angle = Math.toDegrees(Math.atan((rightDistance - leftDistance) / DISTANCE_SENSOR_OFFSET));
        averageDistance = (rightDistance + leftDistance) / 2;
        distanceFromWall = averageDistance * Math.cos(Math.toRadians(angle));
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("right distance", rightDistance);
        telemetry.addData("left distance", leftDistance);
        telemetry.addData("angle", angle);
        telemetry.addData("average distance", averageDistance);
        telemetry.addData("distance from wall", distanceFromWall);
    }
}

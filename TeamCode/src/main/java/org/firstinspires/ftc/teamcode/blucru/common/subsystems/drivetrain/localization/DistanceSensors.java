package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.blucru.common.util.Subsystem;

// not used anymore
public class DistanceSensors implements Subsystem {
    public static double
            DISTANCE_SENSOR_OFFSET = 5.875, // distance between distance sensors in inches
            DISTANCE_SENSOR_MAX = 45; // max distance the distance sensors can read
    DistanceSensor rightDistanceSensor;
    DistanceSensor leftDistanceSensor;

    // kalman filter variables
    public static double Q = 0.4;
    public static double R = 0.4;
    public static int N = 3;

    double rightDistance;
    double leftDistance;
    public double angle; // angle of the robot relative to the wall, 0 being dead on, 90 being parallel turned counter clockwise
    private double averageDistance;
    public double distanceFromWall; // distance from the wall, 0 being touching the wall

    public boolean sensing;

    public DistanceSensors(HardwareMap hardwareMap) {
        // instantiate distance sensors
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

    }

    public void read(double heading) {
        rightDistance = rightDistanceSensor.getDistance(DistanceUnit.INCH);
        leftDistance = leftDistanceSensor.getDistance(DistanceUnit.INCH);

        // if the distance sensors are sensing, calculate the angle and distance from the wall
        if(rightDistance < DISTANCE_SENSOR_MAX && leftDistance < DISTANCE_SENSOR_MAX) {
            sensing = true;
            angle = Math.atan((rightDistance - leftDistance) / DISTANCE_SENSOR_OFFSET);
            averageDistance = (rightDistance + leftDistance) / 2;
            distanceFromWall = getDistanceFromWall(heading);
        } else {
            sensing = false;
        }
    }

    @Override
    public void write() {

    }

    public double getAngleError(double angle) {
        return angle - this.angle;
    }

    public double getDistanceFromWall(double angle) {
        if(sensing)
            return averageDistance * Math.abs(Math.cos(Math.toRadians(angle)));
        else
            return distanceFromWall;
    }

//    public void resetKalmanFilter() {
//        kalmanFilter = new KalmanFilter(Q, R, N);
//        kalmanFilter.setX(getDistanceFromWall());
//    }

    public void setQ(double Q) {this.Q = Q;}
    public void setR(double R) {this.R = R;}
    public void setN(int N) {this.N = N;}

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("distance sensor angle", angle);
        telemetry.addData("average distance", averageDistance);
        telemetry.addData("distance from wall", distanceFromWall);
    }

    public void testTelemetry(Telemetry telemetry) {
//        telemetry.addData("filtered distance", filteredDistance);
        telemetry.addData("right distance", rightDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("left distance", leftDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("sensing", sensing);
    }
}

package org.firstinspires.ftc.teamcode.BluCru.subsystems;


import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BluCru.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class Drivetrain extends SampleMecanumDrive implements Subsystem{
    public static double maxAccelDriveVectorDelta = 8; // magnitude per second
    public static double maxDecelDriveVectorDelta = 20; // magnitude per second
    public static double turnP = 4, turnI = 0.2, turnD = 0.3;
    public static double distanceP = -0.015, distanceI = -0.12, distanceD = -0.12;

    public double drivePower = 0.5;

    private double dt;
    private Pose2d pose;
    private double velocity;
    private double acceleration;
    private Pose2d lastPose;
    private double lastVelocity;
    private double lastTime;

    private double heading;

    private Vector2d lastDriveVector;
    private double lastRotate;

    // heading while facing intake
    public boolean fieldCentric;

    private PIDController turnPID;
    public double targetHeading = 0;

    private PIDController distancePID;

    public Drivetrain(HardwareMap hardwareMap) {
        super(hardwareMap);
        turnPID = new PIDController(turnP, turnI, turnD);
        distancePID = new PIDController(distanceP, distanceI, distanceD);
    }

    public void init() {
        fieldCentric = true;
        resetIMU();
        lastDriveVector = new Vector2d(0,0);
        lastRotate = 0;
        lastPose = new Pose2d(0,0,0);
        lastTime = System.currentTimeMillis();
        lastVelocity = 0;
        heading = getRelativeHeading();
    }

    public void update() {
        updatePoseEstimate();
        dt = System.currentTimeMillis() - lastTime;
        pose = getPoseEstimate();
        velocity = pose.vec().distTo(lastPose.vec()) / dt;
        acceleration = (velocity - lastVelocity) / dt;
        lastPose = pose;
        lastVelocity = velocity;
        lastTime = System.currentTimeMillis();
        heading = getRelativeHeading();
    }

    public void drive(double x, double y, double rotate) {
        Vector2d driveVector = calculateDriveVector(new Vector2d(x, y));

        x = driveVector.getX();
        y = driveVector.getY();

        if (Math.max(Math.max(Math.abs(x), Math.abs(y)), Math.abs(rotate)) > 0.1) {
            setWeightedDrivePower(new Pose2d(x * drivePower, y * drivePower, rotate * drivePower));
        } else {
            setWeightedDrivePower(new Pose2d(0, 0, 0));
        }
    }

    public void driveToHeading(double x, double y, double targetHeading) {
        Vector2d driveVector = calculateDriveVector(new Vector2d(x,y));

        x = driveVector.getX();
        y = driveVector.getY();

        double rotate = getPIDRotate(heading, targetHeading);

        if (Math.max(Math.max(Math.abs(x), Math.abs(y)), Math.abs(rotate)) > 0.1) {
            setWeightedDrivePower(new Pose2d(x * drivePower, y * drivePower, rotate * drivePower));
        } else {
            setWeightedDrivePower(new Pose2d(0, 0, 0));
        }
    }

    public Vector2d calculateDriveVector(Vector2d input) {
        // scale down so magnitude isnt greater than 1
        input = input.div(input.norm()).times(Range.clip(input.norm(), 0, 1));

        // rotate input vector to match robot heading
        if (fieldCentric) {
            input = input.rotated(Math.toRadians(-90) - heading);
        } else {
            input = input.rotated(Math.toRadians(-90));
        }

        // calculate the delta between the last drive vector and the current drive vector
        Vector2d delta = input.minus(lastDriveVector);
        double deltaMag;

        // if we are decelerating, limit the delta to the max decel delta
        if(lastDriveVector.angleBetween(delta) > Math.PI / 2) {
            deltaMag = Range.clip(delta.norm(), 0, (maxDecelDriveVectorDelta / 1000) * dt);
        } else {
            // otherwise, limit the delta to the max accel delta
            deltaMag = Range.clip(delta.norm(), 0, (maxAccelDriveVectorDelta / 1000) * dt);
        }
        // add the delta to the last drive vector
        Vector2d driveVector = lastDriveVector.plus(delta.times(deltaMag));
        lastDriveVector = driveVector;

        return driveVector;
    }

    public void driveToDistance(double x, double y, double targetDistance, double currentDistance, double targetHeading) {
        Vector2d input;
        x = Range.clip(distancePID.calculate(currentDistance, targetDistance), -1, 1);
        if (fieldCentric) {
            input = new Vector2d(x, y).rotated(Math.toRadians(-90) - getRelativeHeading());
        } else {
            input = new Vector2d(x, y).rotated(Math.toRadians(-90));
        }

        x = input.getX();
        y = input.getY();
        double rotate = getPIDRotate(getRelativeHeading(), targetHeading);

        if (Math.max(Math.max(Math.abs(x), Math.abs(y)), Math.abs(rotate)) > 0.05) {
            setWeightedDrivePower(new Pose2d(x * drivePower, y * drivePower, rotate * drivePower));
        } else {
            setWeightedDrivePower(new Pose2d(0, 0, 0));
        }
    }

    public void setDrivePower(double power) {
        drivePower = power;
    }

    public double getPIDRotate(double heading, double target) {
        if(heading - target < -Math.PI) {
            heading += 2*Math.PI;
        } else if(heading - target > Math.PI) {
            heading -= 2*Math.PI;
        }
        return Range.clip(turnPID.calculate(heading, target), -1, 1);
    }

    public double getRelativeHeading() {
        double heading = getExternalHeading();
        if(heading > Math.PI) {
            heading -= 2*Math.PI;
        } else if(heading < -Math.PI) {
            heading += 2*Math.PI;
        }
        return heading;
    }

    // resets IMU (facing forwards)
    public void resetIMU() {
        super.resetIMU();
    }

    public void stop() {
        super.stop();
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("heading", getExternalHeading());
        telemetry.addData("x", getPoseEstimate().getX());
        telemetry.addData("y", getPoseEstimate().getY());
        telemetry.addData("field centric", fieldCentric);
    }

    public void testTelemetry(Telemetry telemetry) {
        telemetry.addData("drive power", drivePower);
        telemetry.addData("dt", dt);
        telemetry.addData("pose", pose);
        telemetry.addData("velocity", velocity);
        telemetry.addData("acceleration", acceleration);
        telemetry.addData("heading", getExternalHeading());
        telemetry.addData("x", getPoseEstimate().getX());
        telemetry.addData("y", getPoseEstimate().getY());
        telemetry.addData("field centric", fieldCentric);
    }
}

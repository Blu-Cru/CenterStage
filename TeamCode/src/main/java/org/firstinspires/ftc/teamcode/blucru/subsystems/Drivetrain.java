package org.firstinspires.ftc.teamcode.blucru.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.states.RobotState;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class Drivetrain extends SampleMecanumDrive implements Subsystem {
    public static double DRIVE_POWER_RETRACT = 0.7, DRIVE_POWER_OUTTAKE = 0.4;

    public static double maxAccelDriveVectorDelta = 5; // magnitude per second at power 1
    public static double maxDecelDriveVectorDelta = 30.0; // magnitude per second at power 1
    public static double turnP = 1.0, turnI = 0, turnD = 0.02;

    public static double distanceP = -0.015, distanceI = -0.12, distanceD = -0.12;
    public static double angleTolerance = 0.5; // radians
    public static double OUTTAKE_DISTANCE;

    public double drivePower = 0.5;
    private double dt;
    private Pose2d pose;
    private double velocity;
    private double acceleration;
    private Pose2d lastPose;
    private double lastVelocity;
    private double lastTime;

    public double heading;

    private Vector2d lastDriveVector;
    private double lastRotate;

    // heading while facing intake
    public boolean fieldCentric;

    private PIDController turnPID;
    public double targetHeading = 0;

    public DistanceSensors distanceSensors;
    private PIDController distancePID;

    public Drivetrain(HardwareMap hardwareMap) {
        super(hardwareMap);
        turnPID = new PIDController(turnP, turnI, turnD);
        distancePID = new PIDController(distanceP, distanceI, distanceD);
        distanceSensors = new DistanceSensors(hardwareMap);
    }

    public void init() {
        fieldCentric = true;
        resetHeading();
        lastDriveVector = new Vector2d(0,0);
        lastRotate = 0;
        lastPose = new Pose2d(0,0,0);
        lastTime = System.currentTimeMillis();
        lastVelocity = 0;
        heading = getRelativeHeading();
    }

    public void read() {
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

    public void write() {

    }

    public void drive(double x, double y, double rotate) {
        Vector2d driveVector = calculateDriveVector(new Vector2d(x, y));

        x = driveVector.getX();
        y = driveVector.getY();

        setWeightedDrivePower(new Pose2d(x * drivePower, y * drivePower, rotate * drivePower));
    }

    public void driveToHeading(double x, double y, double targetHeading) {
        Vector2d driveVector = calculateDriveVector(new Vector2d(x,y));

        x = driveVector.getX();
        y = driveVector.getY();

        double rotate = Range.clip(getPIDRotate(heading, targetHeading), -drivePower, drivePower);

        setWeightedDrivePower(new Pose2d(x * drivePower, y * drivePower, rotate));

    }

    public Vector2d calculateDriveVector(Vector2d input) {
        // scale acceleration to match drive power
        double scaledAccelDelta = maxAccelDriveVectorDelta / drivePower;
        double scaledDecelDelta = maxDecelDriveVectorDelta / drivePower;

        // scale down so magnitude isnt greater than 1
//        input = input.div(input.norm()).times(Range.clip(input.norm(), 0, 1));

        // rotate input vector to match robot heading
        if (fieldCentric) {
            input = input.rotated(-heading);
        } else {
            input = input.rotated(Math.toRadians(-90));
        }

        // calculate the delta between the last drive vector and the current drive vector
        Vector2d delta = input.minus(lastDriveVector);
        double deltaMag;

        // if we are decelerating, limit the delta to the max decel delta
        if(lastDriveVector.norm() > input.norm()) {
            deltaMag = Range.clip(delta.norm(), 0, (scaledDecelDelta * dt / 1000.0));
        } else {
            // otherwise, limit the delta to the max accel delta

            deltaMag = Range.clip(delta.norm(), 0, (scaledAccelDelta * dt / 1000.0));
        }

        Vector2d driveVector;
        // add the delta to the last drive vector
        if(delta.norm() != 0) {
            driveVector = lastDriveVector.plus(delta.div(delta.norm()).times(deltaMag));
        } else {
            driveVector = lastDriveVector;
        }
        lastDriveVector = driveVector;

        return input;
    }

    public void driveToDistanceToHeading(double x, double y, double targetDistance, double targetHeading) {
//        distanceSensors.update();
        Vector2d distanceVector = new Vector2d(x,y);

        Vector2d driveVector = calculateDriveVector(distanceVector);

        double component;
        if(Math.abs(distanceSensors.getAngleError(heading - targetHeading)) < angleTolerance && distanceSensors.sensing) {
            component = Range.clip(distancePID.calculate(distanceSensors.distanceFromWall, targetDistance), -drivePower, drivePower);
            // set component in direction opposite target heading
            driveVector = setComponent(driveVector, component, -(heading - targetHeading));
        }

        x = driveVector.getX();
        y = driveVector.getY();
        double rotate = Range.clip(getPIDRotate(heading, targetHeading), -drivePower, drivePower);

        setWeightedDrivePower(new Pose2d(x * drivePower, y * drivePower, rotate));
    }

    public Vector2d setComponent(Vector2d vector, double component, double angle) {
        vector = vector.rotated(-angle);
        vector = new Vector2d(component, vector.getY());
        return vector.rotated(angle);
    }

    public double getDistanceSensorAngleError(double targetHeading) {
        distanceSensors.read();
        double error = heading - targetHeading - distanceSensors.angle;
        if(error > Math.PI) {
            error -= 2 * Math.PI;
        } else if (error < -Math.PI) {
            error += 2 * Math.PI;
        }

        return Math.abs(error);
    }

    public void setDrivePower(double power) {
        drivePower = Range.clip(power, 0.0, 1.0);
    }

    public void setDistancePID(double p, double i, double d) {
        distancePID.setPID(p, i, d);
    }

    public void setTurnPID(double p, double i, double d) {
        turnPID.setPID(p, i, d);
    }

    public double getPIDRotate(double heading, double target) {
        if(heading - target < -Math.PI) {
            heading += 2*Math.PI;
        } else if(heading - target > Math.PI) {
            heading -= 2 * Math.PI;
        }
        return Range.clip(turnPID.calculate(heading, target), -1, 1);
    }

    public double getRelativeHeading() {
        double heading = getPoseEstimate().getHeading();
        if(heading > Math.PI) {
            heading -= 2*Math.PI;
        } else if(heading < -Math.PI) {
            heading += 2*Math.PI;
        }
        return heading;
    }

    public void setDrivePower(RobotState robotState, Gamepad gamepad1) {
        double drivePowerMultiplier;
        if(gamepad1.left_trigger > 0.1) {
            drivePowerMultiplier = 1.0 - (0.5 * gamepad1.left_trigger);
        } else if (gamepad1.right_trigger > 0.1) {
            drivePowerMultiplier = 1.0 + (0.3 * gamepad1.right_trigger);
        } else {
            drivePowerMultiplier = 1.0;
        }

        double power = 0.0;
        switch (robotState) {
            case RETRACT:
                power = DRIVE_POWER_RETRACT * drivePowerMultiplier;
                break;
            case OUTTAKE:
                power = DRIVE_POWER_OUTTAKE * drivePowerMultiplier;
                break;
        }
        setDrivePower(power);
    }

    // resets IMU (intake facing forwards)
    public void resetHeading() {
        setPoseEstimate(new Pose2d(0,0,Math.toRadians(90)));
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("drive power", drivePower);
        telemetry.addData("heading", getExternalHeading());
        telemetry.addData("x", getPoseEstimate().getX());
        telemetry.addData("y", getPoseEstimate().getY());
        telemetry.addData("field centric", fieldCentric);
    }

    public void testTelemetry(Telemetry telemetry) {
        telemetry.addData("dt", dt);
        telemetry.addData("last drive vector", lastDriveVector);
    }
}

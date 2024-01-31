package org.firstinspires.ftc.teamcode.blucru.common.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.states.Initialization;
import org.firstinspires.ftc.teamcode.blucru.common.states.RobotState;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class Drivetrain extends SampleMecanumDrive implements Subsystem {
    public static double DRIVE_POWER_RETRACT = 0.8, DRIVE_POWER_OUTTAKE = 0.4;

    public static double MAX_ACCEL_DRIVE_DELTA = 5; // magnitude per second at power 1
    public static double MAX_DECEL_DRIVE_DELTA = 30.0; // magnitude per second at power 1
    public static double TURN_P = 1.0, TURN_I = 0, TURN_D = 0.02;

    public static double DISTANCE_P = 0.15, DISTANCE_I = 0, DISTANCE_D = 0.04;
    public static double DISTANCE_ANGLE_TOLERANCE = 0.5; // radians
    public static double HEADING_ANGLE_TOLERANCE = 0.25; // radians

    public static double OUTTAKE_DISTANCE = 3.7;

    public double drivePower = 0.5;
    private double dt;
    private Pose2d pose;
    private double velocity;
    private double acceleration;
    private Pose2d lastPose;
    private double lastVelocity;
    private double lastTime;

    boolean readingDistance; // only used in auto

    public double heading;
    double imuHeading;
    double odoHeading;

    private Vector2d lastDriveVector;
    private double lastRotate;

    // heading while facing intake
    public boolean fieldCentric;
    boolean isTeleOp;

    private PIDController turnPID;
    public double targetHeading = 0;

    public DistanceSensors distanceSensors;
    private PIDController distancePID;

    public Drivetrain(HardwareMap hardwareMap, boolean isTeleOp) {
        super(hardwareMap, isTeleOp);
        this.isTeleOp = isTeleOp;
        turnPID = new PIDController(TURN_P, TURN_I, TURN_D);
        distancePID = new PIDController(DISTANCE_P, DISTANCE_I, DISTANCE_D);
        distanceSensors = new DistanceSensors(hardwareMap);

        readingDistance = false;
    }

    public void init() {
        fieldCentric = true;
        if(isTeleOp) {
            initializePose();
            lastDriveVector = new Vector2d(0,0);
            lastRotate = 0;
            lastPose = new Pose2d(0,0,0);
            lastTime = System.currentTimeMillis();
            heading = getRelativeHeading();
        }
    }

    public void read() {
        if(isTeleOp) {
            updatePoseEstimate();
            dt = System.currentTimeMillis() - lastTime;
            lastTime = System.currentTimeMillis();
            odoHeading = getRelativeHeading();
            imuHeading = getIMUHeading();

            if(Math.abs(odoHeading - imuHeading) > HEADING_ANGLE_TOLERANCE) {
                heading = odoHeading;
            } else {
                heading = imuHeading;
            }
        }

        pose = this.getPoseEstimate();
        if(readingDistance) {
            distanceSensors.read();
        }
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
        double scaledAccelDelta = MAX_ACCEL_DRIVE_DELTA / drivePower;
        double scaledDecelDelta = MAX_DECEL_DRIVE_DELTA / drivePower;

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
        distanceSensors.read();
        Vector2d distanceVector = new Vector2d(x,y);

        Vector2d driveVector = calculateDriveVector(distanceVector);

        double component;
        if(Math.abs(distanceSensors.getAngleError(heading - targetHeading)) < DISTANCE_ANGLE_TOLERANCE && distanceSensors.sensing) {
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

    public double getIMUHeading() {
        double heading = getExternalHeading();
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
        resetIMU();
        setPoseEstimate(new Pose2d(0,0,Math.toRadians(90)));
    }

    public void initializePose() {
        setPoseEstimate(Initialization.POSE);
        setExternalHeading(Initialization.POSE.getHeading());
    }

//    public void startReadingDistance() {
//        readingDistance = true;
//        distanceSensors.resetKalmanFilter();
//    }

    public void stopReadingDistance() {
        readingDistance = false;
    }

    public void telemetry(Telemetry telemetry) {
        if(isTeleOp) {
            telemetry.addData("drive power", drivePower);
            telemetry.addData("field centric", fieldCentric);
        } else {
            telemetry.addData("reading distance", readingDistance);
        }
        telemetry.addData("heading", heading);
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
    }

    public void testTelemetry(Telemetry telemetry) {
        telemetry.addData("dt", dt);
        telemetry.addData("last drive vector", lastDriveVector);
        distanceSensors.telemetry(telemetry);
    }
}

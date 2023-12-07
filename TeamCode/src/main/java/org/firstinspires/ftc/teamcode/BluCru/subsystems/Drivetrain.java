package org.firstinspires.ftc.teamcode.BluCru.subsystems;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BluCru.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;

public class Drivetrain extends SampleMecanumDrive implements Subsystem{
    public static Vector2d flVector = new Vector2d(0.47817, 0.8783);
    public static Vector2d frVector = new Vector2d(-0.47817, 0.8783);
    public static Vector2d blVector = frVector;
    public static Vector2d brVector = flVector;

    public static double maxAcceleration; // inches per second squared
    public static double maxDriveVectorDelta = 4; // magnitude per second

    private double drivePower = 0.5;
    private double dt;
    private Pose2d pose;
    private double velocity;
    private double acceleration;
    private Pose2d lastPose;
    private double lastVelocity;
    private double lastTime;

    private Vector2d lastDriveVector;

    // heading while facing intake
    private boolean fieldCentric;

    private PIDController turnPID;
    public double targetHeading = 0;

    public Drivetrain(HardwareMap hardwareMap) {
        super(hardwareMap);
        turnPID = new PIDController(Constants.turnP, Constants.turnI, Constants.turnD);
    }

    public void init() {
        fieldCentric = true;
        resetIMU();
        lastDriveVector = new Vector2d(0,0);
        lastPose = new Pose2d(0,0,0);
        lastTime = System.currentTimeMillis();
        lastVelocity = 0;
    }

    public void update() {
        dt = System.currentTimeMillis() - lastTime;
        pose = getPoseEstimate();
        velocity = pose.vec().distTo(lastPose.vec()) / dt;
        acceleration = (velocity - lastVelocity) / dt;
        lastPose = pose;
        lastVelocity = velocity;
        lastTime = System.currentTimeMillis();
    }

    public void driveClipAcceleration(Vector2d inputVector, double rotate) {
        // scale down so magnitude isnt greater than 1
        inputVector = inputVector.div(inputVector.norm()).times(Range.clip(inputVector.norm(), 0, 1));

        Vector2d delta = inputVector.minus(lastDriveVector);

        // keep delta magnitude below max drive vector velocity
        double deltaMag = Range.clip(delta.norm() / dt, 0, maxDriveVectorDelta / 1000);

        Vector2d driveVector;
        driveVector = lastDriveVector.plus(delta.times(deltaMag));

        drive(driveVector, rotate);

        lastDriveVector = inputVector;
    }

    public void drive(Vector2d input, double rotate) {
        if (fieldCentric) {
            input = input.rotated(Math.toRadians(-90) - getRelativeHeading());
        } else {
            input = input.rotated(Math.toRadians(-90));
        }

        double x = input.getX();
        double y = input.getY();

        if (Math.max(Math.max(Math.abs(x), Math.abs(y)), Math.abs(rotate)) > 0.1) {
            setWeightedDrivePower(new Pose2d(x * drivePower, y * drivePower, rotate * drivePower));
        } else {
            setWeightedDrivePower(new Pose2d(0, 0, 0));
        }
    }

    public void driveToHeading(double x, double y, double targetHeading) {
        Vector2d input;
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

    // resets heading (do while in scoring position)
    public void resetHeadingOffset() {
        super.setExternalHeading(Math.toRadians(90));
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
}

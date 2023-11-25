package org.firstinspires.ftc.teamcode.BluCru.subsystems;

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
    private double drivePower = 0.5;

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
    }

    public void update() {

    }

    public void drive(double x, double y, double rotate) {
        Vector2d input;
        if (fieldCentric) {
            input = new Vector2d(x, y).rotated(Math.toRadians(-90) - getRelativeHeading());
        } else {
            input = new Vector2d(x, y).rotated(Math.toRadians(-90));
        }

        x = input.getX();
        y = input.getY();

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

    public void resetHeadingOffset() {
        super.setExternalHeading(0);
    }

    public double getPIDRotate(double heading, double target) {
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

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("heading", getExternalHeading());
        telemetry.addData("relative heading", getRelativeHeading());
        telemetry.addData("x", getPoseEstimate().getX());
        telemetry.addData("y", getPoseEstimate().getY());
        telemetry.addData("field centric", fieldCentric);
    }
}

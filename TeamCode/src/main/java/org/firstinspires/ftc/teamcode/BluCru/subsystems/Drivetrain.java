package org.firstinspires.ftc.teamcode.BluCru.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;

public class Drivetrain extends SampleMecanumDrive implements Subsystem{
    private double drivePower = 0.5;

    // heading while facing forwards
    private double headingOffset;
    private boolean fieldCentric;

    public Drivetrain(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void init() {
        headingOffset = getExternalHeading();
        fieldCentric = true;
    }

    public void update() {

    }

    public void drive(double x, double y, double rotate) {
        Vector2d input;
        if (fieldCentric) {
            input = new Vector2d(x, y).rotated(Math.toRadians(-90) + Math.toRadians(getRawExternalHeading()) - Math.toRadians(headingOffset));
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

    public void setDrivePower(double power) {
        drivePower = power;
    }

    public void resetHeadingOffset() {
        this.headingOffset = this.getExternalHeading();
    }

    public double getRelativeHeading() {
        double heading = getExternalHeading() - headingOffset;
        if(heading > Math.PI) {
            heading -= 2*Math.PI;
        } else if(heading < -Math.PI) {
            heading += 2*Math.PI;
        }
        return heading;
    }
}

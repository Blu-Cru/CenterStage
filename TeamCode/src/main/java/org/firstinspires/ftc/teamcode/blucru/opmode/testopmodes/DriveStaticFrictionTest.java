package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import org.firstinspires.ftc.teamcode.blucru.common.util.BCLinearOpMode;

public class DriveStaticFrictionTest extends BCLinearOpMode {
    boolean lastA = false;
    enum DriveType {
        RAW,
        STATIC_FRICTION
    }

    DriveType driveType = DriveType.RAW;

    @Override
    public void initialize() {
        addDrivetrain(true);
        drivetrain.fieldCentric = false;
    }

    @Override
    public void periodic() {
        if(gamepad1.a && !lastA) {
            driveType = driveType == DriveType.RAW ? DriveType.STATIC_FRICTION : DriveType.RAW;
        }
        lastA = gamepad1.a;

        double horz = gamepad1.left_stick_x;
        double vert = -gamepad1.left_stick_y;
        double rot = -gamepad1.right_stick_x;

        if(driveType == DriveType.RAW) {
            drivetrain.drive(horz, vert, rot);
        } else {
            drivetrain.driveStaticFriction(horz, vert, rot);
        }
    }

    public void telemetry() {
        telemetry.addData("drive type", driveType);
        drivetrain.testTelemetry(telemetry);
    }
}

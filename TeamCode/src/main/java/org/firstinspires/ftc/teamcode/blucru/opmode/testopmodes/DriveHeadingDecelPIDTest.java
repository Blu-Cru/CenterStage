package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

public class DriveHeadingDecelPIDTest extends BCLinearOpMode {
    public void initialize() {
        enableFTCDashboard();
        addDrivetrain(false);
    }

    @Override
    public void periodic() {
        double horz = gamepad1.left_stick_x;
        double vert = -gamepad1.left_stick_y;
        double rot = -gamepad1.right_stick_x;

        if(gamepad1.right_stick_button) {
            drivetrain.resetHeading(Math.toRadians(90));
            gamepad1.rumble(100);
        }

        if(gamepad1.b) drivetrain.driveToHeadingScaled(horz, vert, Math.toRadians(180));
        else if (gamepad1.x) drivetrain.driveToHeadingScaledDecel(horz, vert, Math.toRadians(180));
        else drivetrain.teleOpDrive(horz, vert, rot);
    }
}

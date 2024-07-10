package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import org.firstinspires.ftc.teamcode.blucru.opmode.KLinearOpMode;

public class PoseHistoryTest extends KLinearOpMode {
    @Override
    public void initialize() {
        addDrivetrain(true);
    }

    @Override
    public void periodic() {
        double horz = gamepad1.left_stick_x;
        double vert = -gamepad1.left_stick_y;
        double rot = -gamepad1.right_stick_x;

        drivetrain.teleOpDrive(horz, vert, rot);
    }
}

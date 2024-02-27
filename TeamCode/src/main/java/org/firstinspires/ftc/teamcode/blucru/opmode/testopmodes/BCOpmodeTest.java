package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.util.BCLinearOpMode;

@TeleOp(name="BCOpmodeTest", group="test")
public class BCOpmodeTest extends BCLinearOpMode {
    public void initialize() {
        addDrivetrain(true);
        addIntakeWrist();
    }

    public void initLoop() {
        telemetry.addData("testing opmode", "init loop");
        telemetry.update();
    }

    public void read() {
        // drive
        double vert = -gamepad1.left_stick_y;
        double horz = gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        drivetrain.drive(vert, horz, rotate);
    }

    public void write() {

    }

    public void telemetry() {
        telemetry.addData("testing opmode", "telemetry");
    }
}

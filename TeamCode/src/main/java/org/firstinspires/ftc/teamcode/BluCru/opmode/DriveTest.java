package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BluCru.subsystems.Drivetrain;

@TeleOp(name = "drive test", group = "TeleOp")
public class DriveTest extends LinearOpMode {
    Drivetrain drivetrain;

    double vert, horz, rotate;
    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.setDrivePower(0.5);

        waitForStart();
        while(opModeIsActive()) {
            vert = -gamepad1.left_stick_y;
            horz = gamepad1.left_stick_x;
            rotate = -gamepad1.right_stick_x;

            drivetrain.drive(horz, vert, rotate);

            telemetry.addData("target", 0);
            telemetry.addData("current", 0);
            telemetry.addData("power", 0);
            telemetry.update();
        }
    }
}

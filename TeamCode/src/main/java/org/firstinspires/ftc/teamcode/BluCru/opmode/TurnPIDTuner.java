package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BluCru.subsystems.Drivetrain;

@Config
@TeleOp(name = "turn PID tuner", group = "TeleOp")
public class TurnPIDTuner extends LinearOpMode {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double target = 0;
    double vert, horz, rotate;
    Drivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.init();

        waitForStart();

        while(opModeIsActive()) {
            horz = gamepad1.left_stick_x;
            vert = -gamepad1.left_stick_y;
            rotate = -gamepad1.right_stick_x;
            if(gamepad1.right_stick_button) {
                drivetrain.resetHeadingOffset();
            }
            controller.setPID(p, i, d);
            double heading = drivetrain.getRelativeHeading();
            double PIDrotate = controller.calculate(heading, target);

            if(gamepad1.a) {
                drivetrain.drive(horz, vert, PIDrotate);
            } else {
                drivetrain.drive(horz, vert, rotate);
            }

            telemetry.addData("target", target);
            telemetry.addData("current heading", heading);
            telemetry.addData("PID rotate", PIDrotate);
            telemetry.update();
        }
    }
}

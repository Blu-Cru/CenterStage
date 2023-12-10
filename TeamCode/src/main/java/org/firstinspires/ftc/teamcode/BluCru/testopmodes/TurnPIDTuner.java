package org.firstinspires.ftc.teamcode.BluCru.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.BluCru.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Intake;

@Config
@Disabled
@TeleOp(name = "turn PID tuner", group = "TeleOp")
public class TurnPIDTuner extends LinearOpMode {
    private PIDController controller;
    public static double p = 1.2, i = 0, d = 0;
    public static double target = 0;
    double vert, horz, rotate;
    Drivetrain drivetrain;
    Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.init();

        intake = new Intake(hardwareMap, telemetry);
        intake.init();

        waitForStart();

        while(opModeIsActive()) {
            horz = gamepad1.left_stick_x;
            vert = -gamepad1.left_stick_y;
            rotate = -gamepad1.right_stick_x;
            if(gamepad1.right_stick_button) {
                drivetrain.resetIMU();
            }
            controller.setPID(p, i, d);
            double heading = drivetrain.getRelativeHeading();

            if(gamepad1.a) {
                target = 0;
                rotate = getPIDRotate(heading);
            }
            if(gamepad1.b) {
                target = Math.toRadians(90);
                rotate = getPIDRotate(heading);
            }

            drivetrain.drive(new Vector2d(horz, vert), rotate);

            telemetry.addData("target", target);
            telemetry.addData("current heading", heading);
            telemetry.addData("rotate", rotate);
            telemetry.update();
        }
    }

    public double getPIDRotate(double heading) {
        return Range.clip(controller.calculate(heading, target), -1, 1);
    }
}

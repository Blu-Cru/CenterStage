package org.firstinspires.ftc.teamcode.blucru.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.subsystems.Intake;

@Config
@TeleOp(name = "turn PID tuner", group = "TeleOp")
public class TurnPIDTuner extends LinearOpMode {
    public static double p = 1.2, i = 0, d = 0;
    public static double target = 0;
    double vert, horz, rotate;
    Drivetrain drivetrain;
    Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.init();
        drivetrain.drivePower = 1;

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
            drivetrain.setTurnPID(p, i, d);

            if(gamepad1.a) {
                target = 0;
                drivetrain.driveToHeading(horz, vert, target);
            } else if(gamepad1.b) {
                target = Math.toRadians(90);
                drivetrain.driveToHeading(horz, vert, target);
            } else {
                drivetrain.drive(horz, vert, rotate);
            }

            drivetrain.write();
            drivetrain.telemetry(telemetry);
            telemetry.addData("target", target);
            telemetry.update();
        }
    }
}

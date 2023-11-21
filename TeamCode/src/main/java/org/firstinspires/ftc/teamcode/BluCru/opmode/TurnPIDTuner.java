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
    public static double p = 0.01, i = 0, d = 0.0002;
    public static double f = 0.07;
    public static int target = 0;

    Drivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("target", target);
            telemetry.addData("current", 0);
            telemetry.addData("power", 0);
            telemetry.update();
        }
    }
}

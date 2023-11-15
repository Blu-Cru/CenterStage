package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.BluCru.Hardware6417;

@Config
@TeleOp(name = "slide PID tuner", group = "TeleOp")
public class PIDTuner extends LinearOpMode {
    private PIDController controller;
    public static double p = 0.01, i = 0, d = 0.0002;
    public static double f = 0.07;
    public static int target = 0;

    private DcMotorEx slider, auxSlider;
    Hardware6417 robot;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Hardware6417(hardwareMap);

        robot.initSlides();

        waitForStart();

        while(opModeIsActive()) {
            controller.setPID(p, i, d);
            int sliderPos = robot.slider.getCurrentPosition();
            double power = controller.calculate(sliderPos, target) + f;

            robot.setSlidePowers(power);

            telemetry.addData("target", target);
            telemetry.addData("current", sliderPos);
            telemetry.addData("power", power);
            telemetry.update();
        }
    }
}

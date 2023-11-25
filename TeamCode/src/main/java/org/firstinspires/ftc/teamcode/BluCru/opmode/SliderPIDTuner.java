package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.BluCru.Hardware6417;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Intake;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Lift;

@Config
@TeleOp(name = "slide PID tuner", group = "TeleOp")
public class SliderPIDTuner extends LinearOpMode {
    private PIDController controller;
    public static double p = 0.01, i = 0, d = 0.0002;
    public static double f = 0.07;
    public static int target = 0;

    Lift lift;
    Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake = new Intake(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        lift.init();
        intake.init();

        waitForStart();

        while(opModeIsActive()) {
            controller.setPID(p, i, d);
            int sliderPos = lift.getCurrentPos();
            double power = controller.calculate(sliderPos, target) + f;

            lift.setPower(power);

            telemetry.addData("target", target);
            telemetry.addData("current", sliderPos);
            telemetry.addData("power", power);
            telemetry.update();
        }
    }
}

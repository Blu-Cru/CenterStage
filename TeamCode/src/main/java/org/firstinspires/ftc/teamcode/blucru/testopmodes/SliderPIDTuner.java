package org.firstinspires.ftc.teamcode.blucru.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.blucru.subsystems.Intake;
import org.firstinspires.ftc.teamcode.blucru.subsystems.Lift;

@Config
@TeleOp(name = "slide PID tuner", group = "TeleOp")
public class SliderPIDTuner extends LinearOpMode {
    private PIDController controller;
    public static double p = 0.01, i = 0, d = 0.0002;
    public static double f = 0.07;
    public static int target = 0;

    Lift lift;
//    Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        lift.init();
//        intake.init();

        waitForStart();

        while(opModeIsActive()) {
            target = Range.clip(target, Lift.liftMinPos, Lift.liftMaxPos);

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

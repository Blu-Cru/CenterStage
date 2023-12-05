package org.firstinspires.ftc.teamcode.BluCru.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BluCru.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Intake;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Lift;

@Config
@TeleOp(name = "hang PID tuner", group = "TeleOp")
public class HangerPIDTuner extends LinearOpMode {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static int target = 0;

    Hanger hanger;
    Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake = new Intake(hardwareMap, telemetry);
        hanger = new Hanger(hardwareMap, telemetry);
        hanger.init();
        intake.init();

        waitForStart();

        while(opModeIsActive()) {
            controller.setPID(p, i, d);
            int currentPos = hanger.getCurrentPos();
            double power = controller.calculate(currentPos, target);

            hanger.setPower(power);

            telemetry.addData("target", target);
            telemetry.addData("current", currentPos);
            telemetry.addData("power", power);
            telemetry.update();
        }
    }
}

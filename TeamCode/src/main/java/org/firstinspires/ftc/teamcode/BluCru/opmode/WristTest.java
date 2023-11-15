package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BluCru.Hardware6417;

@Config
@TeleOp(name = "wrist test", group = "TeleOp")
public class WristTest extends LinearOpMode {
    public static double position = 0.5;
    private Hardware6417 robot;
    public void runOpMode() throws InterruptedException {
        robot = new Hardware6417(hardwareMap);
        waitForStart();

        while(opModeIsActive()) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            if(gamepad1.a) {
                robot.autoWrist(position);
            } else {
                robot.stopWrist();
            }
            telemetry.addData("position", position);
            telemetry.update();
        }
    }
}

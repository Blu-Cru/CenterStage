package org.firstinspires.ftc.teamcode.BluCru.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.BluCru.Constants;

@Config
@TeleOp(name = "crservo test", group = "TeleOp")
public class CRServoTest extends LinearOpMode {
    String name = "outtake";
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo test = hardwareMap.get(CRServo.class, name);
        waitForStart();
        while(opModeIsActive()) {
            if(Math.abs(gamepad1.left_stick_y) > 0.1) {
                test.setPower(-gamepad1.left_stick_y);
            } else {
                test.setPower(0);
            }

            telemetry.addData("name", name);
            telemetry.addData("power", test.getPower());
            telemetry.update();
        }
    }
}

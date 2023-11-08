package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;


@TeleOp(name = "crservo test", group = "TeleOp")
public class CRServoTest extends LinearOpMode {
    String name = "wheels";
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo test = hardwareMap.get(CRServo.class, name);
        ServoControllerEx controller = (ServoControllerEx) test.getController();
        controller.setServoPwmDisable(test.getPortNumber());
        waitForStart();
        while(opModeIsActive()) {
            if(Math.abs(gamepad1.left_stick_y) > 0.1) {
                controller.setServoPwmEnable(test.getPortNumber());
                test.setPower(gamepad1.left_stick_y/2 + 0.5);
            } else {
                controller.setServoPwmDisable(test.getPortNumber());
            }

            telemetry.addData("name", name);
            telemetry.addData("power", gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}

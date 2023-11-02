package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

public class ServoTest extends LinearOpMode {
    double pos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo test = hardwareMap.get(Servo.class, "test");
        ServoControllerEx controller = (ServoControllerEx) test.getController();
        controller.setServoPwmDisable(test.getPortNumber());
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.right_bumper) {
                controller.setServoPwmEnable(test.getPortNumber());
                test.setPosition(gamepad1.left_stick_y/2 + 0.5);
            } else {
                controller.setServoPwmDisable(test.getPortNumber());
            }

            if(gamepad1.a) {
                pos = test.getPosition();
            }
            telemetry.addData("target pos", gamepad1.left_stick_y);
            telemetry.addData("saved pos", pos);
            telemetry.update();
        }
    }
}

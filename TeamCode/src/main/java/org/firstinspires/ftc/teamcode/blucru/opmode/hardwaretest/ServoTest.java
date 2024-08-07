package org.firstinspires.ftc.teamcode.blucru.opmode.hardwaretest;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name = "servo test", group = "hardware test")
public class ServoTest extends LinearOpMode {
    public static double position = 0.5;
    public static String name = "wrist";
    public static boolean reversed = false;
    ServoImplEx test;
    ServoControllerEx controller;

    @Override
    public void runOpMode() throws InterruptedException {
        updateName();
        updateDirection();
        updateController();
        disable();
        waitForStart();
        while(opModeIsActive()) {
            updateName();
            updateDirection();
//            test.setPwmRange(new PwmControl.PwmRange(500, 2500));
            updateController();

            if(gamepad1.a) {
                controller.pwmEnable();
                test.setPosition(position);
            } else {
                controller.pwmDisable();
            }

            telemetry.addData("name", name);
            telemetry.addData("position", test.getPosition());
            telemetry.update();
        }
    }

    public void updateName() {
        try {
            test = hardwareMap.get(ServoImplEx.class, name);
        } catch (Exception e) {
            telemetry.addLine("ERROR: servo " + name + " not found");
        }
    }

    public void updateDirection() {
        try {
            if(reversed) test.setDirection(Servo.Direction.REVERSE);
            else test.setDirection(Servo.Direction.FORWARD);
        } catch (Exception e) {}
    }

    public void updateController() {
        controller = (ServoControllerEx) test.getController();
    }

    public void disable() {
        controller.setServoPwmDisable(test.getPortNumber());
    }
}

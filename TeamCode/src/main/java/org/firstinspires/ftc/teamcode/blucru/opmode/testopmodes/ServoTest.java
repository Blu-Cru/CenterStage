package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

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
    @Override
    public void runOpMode() throws InterruptedException {
        ServoImplEx test = hardwareMap.get(ServoImplEx.class, name);
        ServoControllerEx controller = (ServoControllerEx) test.getController();
        controller.setServoPwmDisable(test.getPortNumber());
        waitForStart();
        while(opModeIsActive()) {
            test = hardwareMap.get(ServoImplEx.class, name);
            test.setPwmRange(new PwmControl.PwmRange(500, 2500));
            controller = (ServoControllerEx) test.getController();

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
}

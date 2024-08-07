package org.firstinspires.ftc.teamcode.blucru.opmode.hardwaretest;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "crservo test", group = "hardware test")
public class CRServoTest extends LinearOpMode {
    public static String name = "intake roller";
    public static boolean reversed = false;
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

            if(reversed) test.setDirection(DcMotorSimple.Direction.REVERSE);
            else test.setDirection(DcMotorSimple.Direction.FORWARD);

            telemetry.addData("name", name);
            telemetry.addData("power", test.getPower());
            telemetry.update();
        }
    }
}

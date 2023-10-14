package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="MotorTest", group="Linear Opmode")
public class MotorTest extends LinearOpMode{
    DcMotorEx testMotor;
    @Override
    public void runOpMode() {
        testMotor = hardwareMap.get(DcMotorEx.class, "test");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        testMotor.setPower(0);
        testMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()) {
            if(Math.abs(gamepad1.left_stick_y) > 0.1) {
                testMotor.setPower(-gamepad1.left_stick_y);
            } else if(gamepad1.a) {
                testMotor.setPower(0);
            } else if(gamepad1.b) {
                testMotor.setPower(0.5);
            } else if(gamepad1.x) {
                testMotor.setPower(-0.5);
            } else {
                testMotor.setPower(0);
            }
            telemetry.update();
        }
    }
}

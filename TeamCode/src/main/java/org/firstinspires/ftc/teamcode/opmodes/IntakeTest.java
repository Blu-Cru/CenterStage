package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="intakeTest", group="Linear Opmode")
public class IntakeTest extends LinearOpMode{
    DcMotorEx testMotor;
    CRServo testServo;
    @Override
    public void runOpMode() {
        testMotor = hardwareMap.get(DcMotorEx.class, "motor");
        testServo = hardwareMap.get(CRServo.class, "crservo");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        testMotor.setPower(0);
        testMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()) {
            if(Math.abs(gamepad1.left_stick_y) > 0.1) {
                testMotor.setPower(-gamepad1.left_stick_y);
            } else if(gamepad1.b) {
                testMotor.setPower(0.5);
            } else if(gamepad1.x) {
                testMotor.setPower(-0.5);
            } else {
                testMotor.setPower(0);
            }

            if (Math.abs(gamepad1.right_stick_y) > 0.1) {
                testServo.setPower(-gamepad1.right_stick_y);
            } else {
                testServo.setPower(0);
            }
            telemetry.addData("gamepad1.left_stick_y", gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}

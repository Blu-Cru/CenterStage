package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MotorTest extends LinearOpMode {
    double pos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx test = hardwareMap.get(DcMotorEx.class, "test");
        test.setDirection(DcMotorSimple.Direction.FORWARD);
        test.setPower(0);
        test.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        test.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        test.setTargetPosition(0);
        test.setPower(0.5);

        boolean lastLB1 = false;
        boolean lastRB1 = false;
        int delta = 100;
        int pos = 0;

        waitForStart();
        while(opModeIsActive()) {
            delta = (int)(-gamepad1.right_stick_y*200) + 200;
            if(gamepad1.right_bumper && !lastRB1) {
                test.setTargetPosition(test.getCurrentPosition() + 100);
            }
            if(gamepad1.left_bumper && !lastLB1) {
                test.setTargetPosition(test.getCurrentPosition() - 100);
            }
            if(gamepad1.a) {
                pos = test.getCurrentPosition();
            }

            lastLB1 = gamepad1.left_bumper;
            lastRB1 = gamepad1.right_bumper;

            telemetry.addData("target", test.getTargetPosition());
            telemetry.addData("current", test.getCurrentPosition());
            telemetry.addData("delta", delta);
            telemetry.addData("saved pos", pos);
            telemetry.update();
        }
    }
}

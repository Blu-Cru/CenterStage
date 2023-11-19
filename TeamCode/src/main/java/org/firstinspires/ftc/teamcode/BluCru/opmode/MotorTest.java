package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "motor test", group = "TeleOp")
public class MotorTest extends LinearOpMode {
    DcMotorEx[] motors = {};

    String name = "slider";
    double pos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx test = hardwareMap.get(DcMotorEx.class, name);

        for(DcMotorEx motor : motors) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        test.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean lastLB1 = false;
        boolean lastRB1 = false;
        int delta = 100;
        int pos = 0;
        double vert;

        waitForStart();
        while(opModeIsActive()) {
            vert = -gamepad1.left_stick_y;



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
            if(Math.abs(vert) > 0.1) {
                test.setPower(vert);
            } else {
                test.setPower(0);
            }

            lastLB1 = gamepad1.left_bumper;
            lastRB1 = gamepad1.right_bumper;

            telemetry.addData("name", name);
            telemetry.addData("target", test.getTargetPosition());
            telemetry.addData("current", test.getCurrentPosition());
            telemetry.addData("delta", delta);
            telemetry.addData("power", test.getPower());
            telemetry.addData("saved pos", pos);
            telemetry.update();
        }
    }
}

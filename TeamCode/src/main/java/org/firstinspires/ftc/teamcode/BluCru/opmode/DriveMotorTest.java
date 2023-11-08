package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "drive motor test", group = "TeleOp")
public class DriveMotorTest extends LinearOpMode {
    DcMotorEx[] motors = new DcMotorEx[4];

    double pos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, "backRight");
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, "backLeft");

        int index = 0;

        motors[0] = fr;
        motors[1] = fl;
        motors[2] = br;
        motors[3] = bl;

        for(DcMotorEx motor : motors) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        boolean lastLB1 = false;
        boolean lastRB1 = false;
        int delta = 100;
        int pos = 0;
        double vert;

        waitForStart();
        while(opModeIsActive()) {
            vert = -gamepad1.left_stick_y;

            if(gamepad1.a) {
                index = 0;
            }
            if(gamepad1.b) {
                index = 1;
            }
            if(gamepad1.x) {
                index = 2;
            }
            if(gamepad1.y) {
                index = 3;
            }

            if(vert > 0.1) {
                motors[index].setPower(vert);
            } else {
                motors[index].setPower(0);
            }

            lastLB1 = gamepad1.left_bumper;
            lastRB1 = gamepad1.right_bumper;

            telemetry.addData("index", index);
            telemetry.addData("power", motors[index].getPower());
            telemetry.addData("saved pos", pos);
            telemetry.update();
        }
    }
}

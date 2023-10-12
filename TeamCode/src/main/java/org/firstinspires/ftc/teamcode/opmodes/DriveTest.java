package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="DriveTest", group="Linear Opmode")
public class DriveTest extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private Gamepad gamepad1, gamepad2;
    double vert, horz, turn, max, speed, leftFront, leftBack, rightFront, rightBack, mag;
    DcMotorEx frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");

        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {
            vert = -gamepad1.left_stick_y;
            horz = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
            speed = 0.7-(0.4*gamepad1.right_trigger);
            mag = Math.sqrt(Math.pow(vert, 2) + Math.pow(horz, 2) + Math.pow(turn, 2));
            if (mag > 1) {
                mag = 1;
            }

            leftFront = vert + horz + turn;
            leftBack = vert - horz + turn;
            rightFront = vert - horz - turn;
            rightBack = vert + horz - turn;

            max = Math.max(Math.max(Math.abs(leftFront), Math.abs(leftBack)), Math.max(Math.abs(rightFront), Math.abs(rightBack)));
            if(max > 1) {
                leftFront /= max;
                leftBack /= max;
                rightFront /= max;
                rightBack /= max;
            }
            frontLeft.setPower(leftFront*speed * mag);
            frontRight.setPower(rightFront*speed* mag);
            backLeft.setPower(leftBack*speed* mag);
            backRight.setPower(rightBack*speed* mag);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

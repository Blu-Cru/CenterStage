package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.BluCru.Hardware6417;

@TeleOp(name = "drive test", group = "TeleOp")
public class DriveTest extends LinearOpMode {
    Hardware6417 robot;
    double vert, horz, rotate;
    PoseVelocity2d drivePoseVelocity;

    @Override
    public void runOpMode() {
        robot = new Hardware6417(hardwareMap);
        robot.initDrive(new Pose2d(0, 0, 0));

        waitForStart();
        while(opModeIsActive()){
            vert = -Math.pow(gamepad1.left_stick_y, 3);
            horz = Math.pow(gamepad1.left_stick_x, 3);
            rotate = Math.pow(gamepad1.right_stick_x, 3);

            drivePoseVelocity = new PoseVelocity2d(new Vector2d(vert, horz), rotate);

            if(Math.max(Math.max(Math.abs(vert), Math.abs(horz)), Math.abs(rotate)) > 0.1) {
                robot.drive.setDrivePowers(drivePoseVelocity);
            } else {
                robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            }

            telemetry.addData("vert", vert);
            telemetry.addData("horz", horz);
            telemetry.addData("rotate", rotate);
            telemetry.update();
        }
    }
}

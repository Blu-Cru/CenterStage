package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.BluCru.Hardware6417;

public class DriveTest extends LinearOpMode {
    Hardware6417 robot;
    double vert, horz, rotate;

    @Override
    public void runOpMode() {
        robot = new Hardware6417(hardwareMap);
        robot.initDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();
        while(opModeIsActive()){
            vert = -Math.pow(gamepad1.left_stick_y, 3);
            horz = Math.pow(gamepad1.left_stick_x, 3);
            rotate = Math.pow(gamepad1.right_stick_x, 3);

            if(Math.max(Math.max(vert, horz), rotate) > 0.1) {
                robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(vert, horz), rotate));
            } else {
                robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            }
        }
    }
}

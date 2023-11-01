package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BluCru.Hardware6417;

public class DriveTest extends LinearOpMode {
    Hardware6417 robot;

    @Override
    public void runOpMode() {
        robot = new Hardware6417(hardwareMap);
        robot.initDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while(opModeIsActive()){
            robot.drive.driveMecanum(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }
}

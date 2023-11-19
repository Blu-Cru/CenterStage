package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.BluCru.Hardware6417;

@TeleOp(name = "drive test", group = "TeleOp")
public class DriveTest extends LinearOpMode {
    Hardware6417 robot;
    double vert, horz, rotate;
    double heading;
    boolean fieldCentric = false;
    boolean lastLB1 = false;
    double driveSpeed;

    @Override
    public void runOpMode() {
        robot = new Hardware6417(hardwareMap);
        robot.initDrive(new Pose2d(0, 0, 0));
        driveSpeed = 0.5;

        waitForStart();
        //yawOffset = robot.drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        while(opModeIsActive()){
            vert = -Math.pow(gamepad1.left_stick_y, 3);
            horz = Math.pow(gamepad1.left_stick_x, 3);
            rotate = -Math.pow(gamepad1.right_stick_x, 3);

            if(gamepad1.a) {
                robot.drive.resetHeadingOffset();
            }

            if(gamepad1.left_bumper && !lastLB1) {
                fieldCentric = !fieldCentric;
            }

            robot.holonomicDrive(horz, vert, rotate, driveSpeed, heading);

            telemetry.addData("field centric", fieldCentric);
            telemetry.addData("yaw pitch roll", robot.drive.imu.getRobotYawPitchRollAngles());
            telemetry.addData("yaw", robot.drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("vert", vert);
            telemetry.addData("horz", horz);
            telemetry.addData("rotate", rotate);
            telemetry.update();
        }
    }


}

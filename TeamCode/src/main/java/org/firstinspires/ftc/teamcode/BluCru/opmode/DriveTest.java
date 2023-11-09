package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.BluCru.Hardware6417;

@TeleOp(name = "drive test", group = "TeleOp")
public class DriveTest extends LinearOpMode {
    Hardware6417 robot;
    double vert, horz, rotate;
    PoseVelocity2d drivePoseVelocity;
    double yawOffset, yaw;
    boolean fieldCentric = false;
    boolean lastLB1 = false;

    @Override
    public void runOpMode() {
        robot = new Hardware6417(hardwareMap);
        robot.initDrive(new Pose2d(0, 0, 0));

        waitForStart();
        //yawOffset = robot.drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        while(opModeIsActive()){
            vert = -Math.pow(gamepad1.left_stick_y, 3);
            horz = Math.pow(gamepad1.left_stick_x, 3);
            rotate = -Math.pow(gamepad1.right_stick_x, 3);
            //yaw = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            drivePoseVelocity = new PoseVelocity2d(new Vector2d(vert, horz), rotate);

            if(gamepad1.a) {
                yawOffset = yaw;
            }

            if(gamepad1.left_bumper && !lastLB1) {
                fieldCentric = !fieldCentric;
            }

            robot.holonomicDrive(vert, horz, rotate, 1, yaw);

            telemetry.addData("field centric", fieldCentric);
            //telemetry.addData("yaw", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            //telemetry.addData("yaw offset", yawOffset);
            telemetry.addData("vert", vert);
            telemetry.addData("horz", horz);
            telemetry.addData("rotate", rotate);
            telemetry.update();
        }
    }


}

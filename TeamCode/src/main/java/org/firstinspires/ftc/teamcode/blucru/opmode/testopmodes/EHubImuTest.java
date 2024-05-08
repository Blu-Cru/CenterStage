package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@TeleOp(name="EHub IMU Test", group="test")
public class EHubImuTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        Drivetrain dt;

        IMU imu = hardwareMap.get(IMU.class, "e hub imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
//        dt = new Drivetrain(hardwareMap, true);
//        dt.init();
//        dt.fieldCentric = false;
        waitForStart();
        while (opModeIsActive()) {
//            dt.read();
//            double horz = gamepad1.left_stick_x;
//            double vert = -gamepad1.left_stick_y;
//            double rot = -gamepad1.right_stick_y;
//
//            dt.teleOpDrive(horz, vert, rot);

            if(gamepad1.right_stick_button) {
                imu.resetDeviceConfigurationForOpMode();
                imu.resetYaw();
            }


            YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Yaw", ypr.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch", ypr.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll", ypr.getRoll(AngleUnit.DEGREES));
            telemetry.update();
//            dt.write();
        }
    }
}

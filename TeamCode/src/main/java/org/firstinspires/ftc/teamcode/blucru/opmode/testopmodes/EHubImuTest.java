package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@TeleOp(name="EHub IMU Test", group="test")
public class EHubImuTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        IMU imu = hardwareMap.get(IMU.class, "e hub imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);
        waitForStart();
        while (opModeIsActive()) {
            YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Yaw", ypr.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch", ypr.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll", ypr.getRoll(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}

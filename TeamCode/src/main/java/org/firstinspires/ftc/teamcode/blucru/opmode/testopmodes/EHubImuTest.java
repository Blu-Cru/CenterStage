package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled
@TeleOp(name="EHub IMU Test", group="test")
public class EHubImuTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        IMU imu = hardwareMap.get(IMU.class, "e hub imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        waitForStart();
        while (opModeIsActive()) {

            if(gamepad1.right_stick_button) {
                imu.resetDeviceConfigurationForOpMode();
                imu.resetYaw();
            }

            YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Yaw", ypr.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch", ypr.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll", ypr.getRoll(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}

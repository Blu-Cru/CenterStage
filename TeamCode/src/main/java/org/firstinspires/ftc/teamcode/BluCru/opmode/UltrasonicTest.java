package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Ultrasonic Test", group = "Neil")
public class UltrasonicTest extends LinearOpMode {
    ModernRoboticsI2cRangeSensor two;
    @Override
    public void runOpMode() throws InterruptedException {
        two = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "two");
        telemetry.addData("Status", "Initialized");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("two: ", two.getDistance(DistanceUnit.INCH));

            telemetry.update();
        }
    }
}

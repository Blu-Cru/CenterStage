package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Distance Test", group = "Neil")
public class DistanceTest extends LinearOpMode {
    DistanceSensor zero;
    DistanceSensor one;
    @Override
    public void runOpMode() throws InterruptedException {
        zero = hardwareMap.get(DistanceSensor.class, "zero");
        one = hardwareMap.get(DistanceSensor.class, "one");
        telemetry.addData("Status", "Initialized");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("zero: ", zero.getDistance(DistanceUnit.INCH));
            telemetry.addData("one: ", one.getDistance(DistanceUnit.INCH));

            telemetry.update();
        }
    }
}

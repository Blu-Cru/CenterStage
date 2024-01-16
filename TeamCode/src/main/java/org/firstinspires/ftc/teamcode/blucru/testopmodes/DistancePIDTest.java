package org.firstinspires.ftc.teamcode.blucru.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.blucru.subsystems.Drivetrain;

@Config
@TeleOp(name = "Distance test", group = "TeleOp")
public class DistancePIDTest extends LinearOpMode {
    public static double distanceP = -0.12;
    public static double distanceI = -0.12;
    public static double distanceD = -0.02;
    public static double targetDistance = 5;
    Drivetrain drivetrain;
    DistanceSensors distanceSensors;

    double vert, horz, rotate;

    double targetHeading;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap);
        distanceSensors = new DistanceSensors(hardwareMap);
        drivetrain.init();

        waitForStart();

        while(opModeIsActive()) {
            vert = -gamepad1.left_stick_y;
            horz = gamepad1.left_stick_x;
            rotate = -gamepad1.right_stick_x;

            drivetrain.distanceSensors.write();

            if(gamepad1.left_bumper && gamepad1.right_bumper) {
                drivetrain.resetIMU();
            }

            if(gamepad1.b) {
                targetHeading = Math.toRadians(90);
                drivetrain.driveToHeading(horz, vert, targetHeading);
            } else if(gamepad1.x) {
                targetHeading = Math.toRadians(-90);
                drivetrain.driveToHeading(horz, vert, targetHeading);
            } else if(gamepad1.a) {
                targetHeading = Math.toRadians(90);
                drivetrain.driveToDistanceToHeading(horz, vert, targetDistance, targetHeading);
            } else {
                drivetrain.drive(horz, vert, rotate);
            }

            drivetrain.setDistancePID(distanceP, distanceI, distanceD);

            distanceSensors.telemetry(telemetry);
            telemetry.addData("heading error", drivetrain.getDistanceSensorAngleError(targetHeading));
            telemetry.update();
        }
    }
}

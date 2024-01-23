package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Drivetrain;

@Config
@TeleOp(name = "Distance test", group = "TeleOp")
public class DistancePIDTest extends LinearOpMode {
    public static double distanceP = 0.12;
    public static double distanceI = 0;
    public static double distanceD = 0;
    public static double targetDistance = 10;
    Drivetrain drivetrain;

    double vert, horz, rotate;

    double targetHeading;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.init();

        waitForStart();

        while(opModeIsActive()) {
            drivetrain.read();

            vert = -gamepad1.left_stick_y;
            horz = gamepad1.left_stick_x;
            rotate = -gamepad1.right_stick_x;


            if(gamepad1.right_stick_button) {
                drivetrain.resetHeading();
                gamepad1.rumble(100);
            }

            if(gamepad1.b) {
                targetHeading = Math.toRadians(180);
                if(gamepad1.left_bumper) {
                    drivetrain.driveToDistanceToHeading(horz, vert, targetDistance, targetHeading);
                } else {
                    drivetrain.driveToHeading(horz, vert, targetHeading);
                }
            } else if(gamepad1.x) {
                targetHeading = Math.toRadians(0);
                if(gamepad1.left_bumper) {
                    drivetrain.driveToDistanceToHeading(horz, vert, targetDistance, targetHeading);
                } else {
                    drivetrain.driveToHeading(horz, vert, targetHeading);
                }
            } else if(gamepad1.a) {
                targetHeading = Math.toRadians(90);
                drivetrain.driveToDistanceToHeading(horz, vert, targetDistance, targetHeading);
            } else {
                drivetrain.drive(horz, vert, rotate);
            }

            drivetrain.setDistancePID(distanceP, distanceI, distanceD);

            drivetrain.write();

            drivetrain.distanceSensors.telemetry(telemetry);
            drivetrain.distanceSensors.testTelemetry(telemetry);
            telemetry.addData("heading error", drivetrain.getDistanceSensorAngleError(targetHeading));
            telemetry.update();
        }
    }
}

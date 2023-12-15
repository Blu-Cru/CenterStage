package org.firstinspires.ftc.teamcode.BluCru.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BluCru.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Drivetrain;

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

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap);
        distanceSensors = new DistanceSensors(hardwareMap);
        drivetrain.init();
        distanceSensors.init();

        waitForStart();

        while(opModeIsActive()) {
            vert = -gamepad1.left_stick_y;
            horz = gamepad1.left_stick_x;
            rotate = -gamepad1.right_stick_x;

            distanceSensors.update();

            if(gamepad1.left_bumper && gamepad1.right_bumper) {
                drivetrain.resetIMU();
            }

            if(gamepad1.b) {
                drivetrain.driveToHeading(horz, vert, Math.toRadians(90));
            } else if(gamepad1.x) {
                drivetrain.driveToHeading(horz, vert, Math.toRadians(-90));
            } else if(gamepad1.a) {
                drivetrain.driveToDistanceToHeading(horz, vert, distanceSensors.distanceFromWall, targetDistance, Math.toRadians(90));
            } else {
                drivetrain.drive(horz, vert, rotate);
            }

            drivetrain.setDistancePID(distanceP, distanceI, distanceD);

            distanceSensors.telemetry(telemetry);
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.BluCru.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BluCru.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Drivetrain;

public class DistancePIDTest extends LinearOpMode {
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
                drivetrain.resetHeadingOffset();
            }

            if(gamepad1.b) {
                drivetrain.driveToHeading(horz, vert, Math.toRadians(90));
            } else if(gamepad1.x) {
                drivetrain.driveToHeading(horz, vert, Math.toRadians(-90));
            } else if(gamepad1.a) {
                drivetrain.driveToDistance(horz, vert, 5, distanceSensors.averageDistance, 90);
            } else {
                drivetrain.drive(horz, vert, rotate);
            }

            distanceSensors.telemetry(telemetry);
            telemetry.update();
        }
    }
}

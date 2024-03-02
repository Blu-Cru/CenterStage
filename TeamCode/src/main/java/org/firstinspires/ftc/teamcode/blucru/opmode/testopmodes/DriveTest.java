package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;

@TeleOp(name = "drive test", group = "test")
public class DriveTest extends LinearOpMode {
    Robot robot;
    Drivetrain drivetrain;

    double vert, horz, rotate;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        drivetrain = robot.addDrivetrain(true);
        Intake intake = robot.addIntake();

        robot.init();

        drivetrain.fieldCentric = true;
        drivetrain.drivePower = 0.8;

        waitForStart();


        while(opModeIsActive()) {
            robot.read();

            vert = -gamepad1.left_stick_y;
            horz = gamepad1.left_stick_x;
            rotate = -gamepad1.right_stick_x;

            if(gamepad1.right_stick_button) {
                drivetrain.resetHeading(Math.toRadians(90));
                gamepad1.rumble(150);
            }

            if(gamepad1.b) {
                if(gamepad1.left_bumper) {
                    drivetrain.driveToDistanceToHeading(horz, vert, 10, Math.toRadians(180));
                } else {
                    drivetrain.driveToHeading(horz, vert, Math.toRadians(180));
                }
            } else if (gamepad1.x) {
                if(gamepad1.left_bumper) {
                    drivetrain.driveToDistanceToHeading(horz, vert, 10, 0);
                } else {
                    drivetrain.driveToHeading(horz, vert, 0);
                }
            } else {
                drivetrain.driveMaintainHeading(horz, vert, rotate);
            }

            robot.write();
            robot.telemetry(telemetry);
            drivetrain.testTelemetry(telemetry);
            telemetry.update();
        }
    }
}

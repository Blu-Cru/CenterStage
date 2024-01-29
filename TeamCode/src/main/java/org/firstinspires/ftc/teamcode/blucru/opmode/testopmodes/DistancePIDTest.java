package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

@Config
@TeleOp(name = "Distance test", group = "TeleOp")
public class DistancePIDTest extends LinearOpMode {
    public static double Q = 0.3;
    public static double R = 0.3;
    public static int N = 3;
    public static double distanceP = 0.12;
    public static double distanceI = 0;
    public static double distanceD = 0;
    public static double targetDistance = 10;
    
    Robot robot;

    double vert, horz, rotate;

    double targetHeading;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.addDrivetrain();
        Intake intake = robot.addIntake();

        drivetrain.distanceSensors.setQ(Q);
        drivetrain.distanceSensors.setR(R);
        drivetrain.distanceSensors.setN(N);

        robot.init();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while(opModeIsActive()) {
            robot.read();

            vert = -gamepad1.left_stick_y;
            horz = gamepad1.left_stick_x;
            rotate = -gamepad1.right_stick_x;

            if(gamepad1.right_stick_button) {
                drivetrain.resetHeading();
                gamepad1.rumble(100);
            }

            if(gamepad1.dpad_up) {
                drivetrain.startReadingDistance();
            }
            if(gamepad1.dpad_down) {
                drivetrain.stopReadingDistance();
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

            robot.write();

            robot.telemetry(telemetry);
            drivetrain.distanceSensors.testTelemetry(telemetry);
            telemetry.addData("heading error", drivetrain.getDistanceSensorAngleError(targetHeading));
            telemetry.update();
        }
    }
}

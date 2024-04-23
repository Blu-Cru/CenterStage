package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

@Disabled
@TeleOp(name = "Distance test", group = "test")
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
        Robot robot = Robot.getInstance();
        Drivetrain drivetrain = robot.addDrivetrain(true);
        Intake intake = robot.addIntake();

//        drivetrain.distanceSensors.setQ(Q);
//        drivetrain.distanceSensors.setR(R);

        robot.init();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while(opModeIsActive()) {
            robot.read();

            vert = -gamepad1.left_stick_y;
            horz = gamepad1.left_stick_x;
            rotate = -gamepad1.right_stick_x;

            if(gamepad1.right_stick_button) {
                drivetrain.resetHeading(Math.toRadians(90));
                gamepad1.rumble(100);
            }

//            if(gamepad1.dpad_up) {
//                drivetrain.startReadingDistance();
//            }
//            if(gamepad1.dpad_down) {
//                drivetrain.stopReadingDistance();
//            }

            if(gamepad1.b) {
                targetHeading = Math.toRadians(180);
                if(gamepad1.left_bumper) {
//                    drivetrain.driveToDistanceToHeading(horz, vert, targetDistance, targetHeading);
                } else {
                    drivetrain.driveToHeading(horz, vert, targetHeading);
                }
            } else if(gamepad1.x) {
                targetHeading = Math.toRadians(0);
                if(gamepad1.left_bumper) {
//                    drivetrain.driveToDistanceToHeading(horz, vert, targetDistance, targetHeading);
                } else {
                    drivetrain.driveToHeading(horz, vert, targetHeading);
                }
            } else if(gamepad1.a) {
                targetHeading = Math.toRadians(90);
//                drivetrain.driveToDistanceToHeading(horz, vert, targetDistance, targetHeading);
            } else {
                drivetrain.drive(horz, vert, rotate);
            }

//            drivetrain.setDistancePID(distanceP, distanceI, distanceD);

            robot.write();

            robot.telemetry(telemetry);
//            drivetrain.distanceSensors.testTelemetry(telemetry);
//            telemetry.addData("heading error", drivetrain.getDistanceSensorAngleError(targetHeading));
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.DrivetrainMigrated;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

@Config
@TeleOp(name = "turn PID tuner", group = "tuner")
public class TurnPIDTuner extends LinearOpMode {
    public static double p = 1.2, i = 0, d = 0;
    public static double target = 0;
    double vert, horz, rotate;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot robot = Robot.getInstance();
        DrivetrainMigrated drivetrain = robot.addDrivetrain(true);
        Intake intake = robot.addIntake();

        robot.init();

        drivetrain.drivePower = 1;

        waitForStart();

        while(opModeIsActive()) {
            robot.read();

            horz = gamepad1.left_stick_x;
            vert = -gamepad1.left_stick_y;
            rotate = -gamepad1.right_stick_x;
            if(gamepad1.right_stick_button) {
                drivetrain.resetHeading(Math.toRadians(90));
                gamepad1.rumble(100);
            }
            drivetrain.setTurnPID(p, i, d);

            if(gamepad1.a) {
                target = Math.toRadians(-90);
                drivetrain.driveToHeading(horz, vert, target);
            } else if(gamepad1.b) {
                target = 0;
                drivetrain.driveToHeading(horz, vert, target);
            } else if(gamepad1.y) {
                target = Math.toRadians(90);
                drivetrain.driveToHeading(horz, vert, target);
            } else if(gamepad1.x) {
                target = Math.toRadians(180);
                drivetrain.driveToHeading(horz, vert, target);
            } else {
                drivetrain.drive(horz, vert, rotate);
            }

            robot.write();
            robot.telemetry(telemetry);
            telemetry.addData("target", target);
            telemetry.update();
        }
    }
}

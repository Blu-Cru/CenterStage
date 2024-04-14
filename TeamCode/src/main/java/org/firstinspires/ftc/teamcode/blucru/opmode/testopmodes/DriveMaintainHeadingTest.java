package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

@Disabled
@TeleOp(name = "Drive Maintain Heading Test", group = "test")
public class DriveMaintainHeadingTest extends LinearOpMode {
    Robot robot;
    Drivetrain drivetrain;
    IntakeWrist intakeWrist;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = Robot.getInstance();
        drivetrain = robot.addDrivetrain(true);
        intakeWrist = robot.addIntakeWrist();
        robot.init();
        waitForStart();
        while(opModeIsActive()) {
            double horz = gamepad1.left_stick_x;
            double vert = -gamepad1.left_stick_y;
            double rotate = -gamepad1.right_stick_x;

            if(gamepad1.right_stick_button) {
                drivetrain.resetHeading(90);
                gamepad1.rumble(150);
            }

            drivetrain.teleOpDrive(horz, vert, rotate);

            robot.read();
            robot.write();
            robot.telemetry(telemetry);
            telemetry.update();
        }
    }
}

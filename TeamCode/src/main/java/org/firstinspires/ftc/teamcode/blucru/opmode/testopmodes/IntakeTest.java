package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.blucru.common.states.RobotState;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

@TeleOp(name = "intake test", group = "TeleOp")
public class IntakeTest extends LinearOpMode {
    Intake intake;
    Drivetrain drivetrain;
    double horz, vert, rotate;

    Gamepad lastGamepad1;
    Gamepad lastGamepad2;

    RobotState robotState;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.addDrivetrain(true);
        Intake intake = robot.addIntake();

        drivetrain.setDrivePower(0.8);

        lastGamepad1 = new Gamepad();
        lastGamepad2 = new Gamepad();

        robot.init();

        waitForStart();
        while(opModeIsActive()) {
            robot.read();

            vert = Math.pow(-gamepad1.left_stick_y, 3);
            horz = Math.pow(gamepad1.left_stick_x, 3);
            rotate = Math.pow(-gamepad1.right_stick_x, 3);

            if(gamepad1.right_stick_button) {
                drivetrain.resetHeading();
                gamepad1.rumble(150);
            }

            if(gamepad1.b) {
                drivetrain.driveToHeading(horz, vert, Math.toRadians(180));
            } else if (gamepad1.x) {
                drivetrain.driveToHeading(horz, vert, 0);
            } else {
                drivetrain.drive(horz, vert, rotate);
            }

            if(gamepad2.left_trigger > 0.1) {
                intake.setIntakePower(gamepad2.left_trigger);
            } else if (gamepad2.right_trigger > 0.1){
                intake.setIntakePower(-gamepad2.right_trigger);
            } else {
                intake.setIntakePower(0);
            }

            if(gamepad2.a) {
                intake.downIntakeWrist();
            } else {
                intake.retractIntakeWrist();
            }

            write(robot);
        }
    }

    public void write(Robot robot) {
        robot.write();

        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);

        telemetry.addData("robot state", robotState);
        robot.telemetry(telemetry);
        telemetry.update();
    }
}

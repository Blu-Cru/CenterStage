package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.blucru.common.states.RobotState;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

@Config
@TeleOp(name = "intake test", group = "TeleOp")
public class IntakeTest extends LinearOpMode {
    public static int stackHeight = 4;
    Robot robot;
    Intake intake;
    Drivetrain drivetrain;
    double horz, vert, rotate;

    RobotState robotState;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        drivetrain = robot.addDrivetrain(true);
        intake = robot.addIntake();

        drivetrain.setDrivePower(0.8);

        robot.init();

        waitForStart();
        while(opModeIsActive()) {
            robot.read();

            vert = Math.pow(-gamepad1.left_stick_y, 3);
            horz = Math.pow(gamepad1.left_stick_x, 3);
            rotate = Math.pow(-gamepad1.right_stick_x, 3);

            if(gamepad1.right_stick_button) {
                drivetrain.resetHeading(90);
                gamepad1.rumble(150);
            }

            if(gamepad1.b) drivetrain.driveToHeading(horz, vert, Math.toRadians(180));
            else if (gamepad1.x) drivetrain.driveToHeading(horz, vert, 0);
            else drivetrain.drive(horz, vert, rotate);


            if(gamepad1.left_trigger > 0.1) intake.setIntakePower(gamepad1.left_trigger);
            else if (gamepad1.right_trigger > 0.1) intake.setIntakePower(-gamepad1.right_trigger);
            else intake.setIntakePower(0);

            if(gamepad1.a) intake.downIntakeWrist();
            else if(gamepad1.y) intake.dropToStack(stackHeight);
            else intake.retractIntakeWrist();

            write();
        }
    }

    public void write() {
        robot.write();

        telemetry.addData("robot state", robotState);
        robot.telemetry(telemetry);
        telemetry.update();
    }
}

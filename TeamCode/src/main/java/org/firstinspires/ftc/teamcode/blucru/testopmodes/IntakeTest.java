package org.firstinspires.ftc.teamcode.blucru.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.blucru.Constants;
import org.firstinspires.ftc.teamcode.blucru.states.RobotState;
import org.firstinspires.ftc.teamcode.blucru.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.subsystems.Intake;

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
        intake = new Intake(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        intake.init();
        drivetrain.init();

        lastGamepad1 = new Gamepad();
        lastGamepad2 = new Gamepad();

        robotState = RobotState.RETRACT;

        waitForStart();
        while(opModeIsActive()) {
            intake.read();
            drivetrain.read();

            drivetrain.setDrivePower(robotState, gamepad1);

            vert = Math.pow(-gamepad1.left_stick_y, 3);
            horz = Math.pow(gamepad1.left_stick_x, 3);
            rotate = Math.pow(-gamepad1.right_stick_x, 3);

            if(gamepad1.right_stick_button) {
                drivetrain.resetIMU();
                gamepad1.rumble(150);
            }

            if(gamepad1.b) {
                drivetrain.driveToHeading(horz, vert, 0);
            } else if (gamepad1.x) {
                drivetrain.driveToHeading(horz, vert, Math.toRadians(180));
            } else {
                drivetrain.drive(horz, vert, rotate);
            }

            switch(robotState) {
                case RETRACT:
                    intake.intakePower = 0;
                    intake.retractIntakeWrist();

                    // left trigger pressed, go to intake
                    if(gamepad1.left_trigger > 0.1 && !(lastGamepad1.left_trigger > 0.1)) {
                        robotState = RobotState.INTAKE;
                        intake.intakePower = gamepad1.left_trigger;
                    }
                    // right trigger pressed, go to intake
                    if(gamepad1.right_trigger > 0.1 && !(lastGamepad1.right_trigger > 0.1)) {
                        robotState = RobotState.INTAKE;
                        intake.intakePower = -gamepad1.right_trigger;
                    }
                    break;
                case INTAKE:
                    if(gamepad1.left_trigger > 0.1) {
                        intake.intakePower = gamepad1.left_trigger;
                    } else if (gamepad1.right_trigger > 0.1){
                        intake.intakePower = -gamepad1.right_trigger;
                    } else {
                        robotState = RobotState.RETRACT;
                        intake.intakePower = 0;
                    }

                    if(gamepad1.a) {
                        intake.downIntakeWrist();
                    } else if(gamepad1.y) {
                        intake.setIntakeWristTargetAngle(Intake.WRIST_STACK3_DEG);
                    } else {
                        intake.retractIntakeWrist();
                    }
                    break;
            }

            write();
        }
    }

    public void write() {
        intake.write();
        drivetrain.write();

        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);

        telemetry.addData("robot state", robotState);
        intake.telemetry(telemetry);
        telemetry.update();
    }
}

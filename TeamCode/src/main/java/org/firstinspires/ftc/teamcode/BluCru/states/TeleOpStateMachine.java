package org.firstinspires.ftc.teamcode.BluCru.states;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BluCru.Constants;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Robot;

public class TeleOpStateMachine {
    private Robot robot;
    private RobotState robotState;

    private Gamepad lastGamepad1;
    private Gamepad lastGamepad2;

    private ElapsedTime liftRetractTimer;

    public TeleOpStateMachine(Robot robot) {
        this.robot = robot;
        robotState = RobotState.RETRACT;
        liftRetractTimer = new ElapsedTime();

        lastGamepad1 = new Gamepad();
        lastGamepad2 = new Gamepad();
    }

// changes states based on gamepad input
    public void updateStates(Gamepad gamepad1, Gamepad gamepad2) {
        switch (robotState) {
            case RETRACT:
                if(gamepad2.a && robot.lift.getCurrentPos() < Constants.sliderIntakeDelta) {
                    robotState = RobotState.INTAKE;
                }
                if(gamepad2.b) {
                    robot.lift.setTargetPos(Constants.sliderLowPos);
                    robotState = RobotState.LIFTING;
                }
                if(gamepad2.x) {
                    robot.lift.setTargetPos(Constants.sliderMedPos);
                    robotState = RobotState.LIFTING;
                }
                if(gamepad2.y) {
                    robot.lift.setTargetPos(Constants.sliderHighPos);
                    robotState = RobotState.LIFTING;
                }
                break;
            case INTAKE:
                if(!gamepad2.a && lastGamepad2.a) {
                    robotState = RobotState.RETRACT;
                }
                break;
            case LIFTING:
                if(robot.lift.getCurrentPos() > Constants.sliderWristClearPos) {
                    robotState = RobotState.OUTTAKE;
                }
                if(gamepad2.a) {
                    robot.lift.resetSliderStallTimer();
                    robotState = RobotState.RETRACT;
                }
                break;
            case OUTTAKE:
                if(gamepad2.a && !lastGamepad2.a) {
                    liftRetractTimer.reset();
                    robotState = RobotState.PREPARE_TO_RETRACT;
                }
                break;
            case PREPARE_TO_RETRACT:
                if(gamepad2.b) {
                    robot.lift.setTargetPos(Constants.sliderLowPos);
                    robotState = RobotState.LIFTING;
                }
                if(gamepad2.x) {
                    robot.lift.setTargetPos(Constants.sliderMedPos);
                    robotState = RobotState.LIFTING;
                }
                if(gamepad2.y) {
                    robot.lift.setTargetPos(Constants.sliderHighPos);
                    robotState = RobotState.LIFTING;
                }
                break;
        }

        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);
    }

// moves robot based on state and gamepad input
    public void updateRobot(Gamepad gamepad1, Gamepad gamepad2) {
        switch (robotState) {
            case RETRACT:
                robot.lift.setTargetPos(Constants.sliderIntakePos);
                robot.drivetrain.setDrivePower(Constants.driveSpeedRetract);
                break;
            case INTAKE:
                robot.lift.setTargetPos(Constants.sliderIntakePos);
                robot.drivetrain.setDrivePower(Constants.driveSpeedIntake);
                break;
            case LIFTING:
                robot.drivetrain.setDrivePower(Constants.driveSpeedOuttake);
                break;
            case OUTTAKE:
                robot.drivetrain.setDrivePower(Constants.driveSpeedOuttake);
                break;
            case PREPARE_TO_RETRACT:
                robot.drivetrain.setDrivePower(Constants.driveSpeedOuttake);
                break;
        }

        robot.lift.update();

        // driving
        // resets heading offset (face forwards)
        if(gamepad1.right_stick_button) {
            robot.drivetrain.resetHeadingOffset();
        }
        robot.drivetrain.drive(Math.pow(gamepad1.left_stick_x, 3), Math.pow(-gamepad1.left_stick_y, 3), Math.pow(-gamepad1.right_stick_x, 3));
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("robot state", robotState);
    }
}

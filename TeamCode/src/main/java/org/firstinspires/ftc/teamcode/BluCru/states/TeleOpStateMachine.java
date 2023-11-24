package org.firstinspires.ftc.teamcode.BluCru.states;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BluCru.Constants;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Robot;

public class TeleOpStateMachine {
    private Robot robot;
    private RobotState robotState;

    private Gamepad lastGamepad1;
    private Gamepad lastGamepad2;

    public TeleOpStateMachine(Robot robot) {
        this.robot = robot;
        robotState = RobotState.RETRACT;

        lastGamepad1 = new Gamepad();
        lastGamepad2 = new Gamepad();
    }

// changes states based on gamepad input
    public void updateStates(Gamepad gamepad1, Gamepad gamepad2) {
        switch (robotState) {
            case RETRACT:
                if(gamepad2.a && robot.lift.getCurrentPos() < Constants.sliderIntakeDelta) {
                    robotState = RobotState.INTAKE;
                    robot.intake.wristState = WristState.INTAKE;
                }
                if(gamepad2.b) {
                    robot.lift.liftState = LiftState.AUTO;
                    robot.lift.setTargetPos(Constants.sliderLowPos);
                    robotState = RobotState.LIFTING;
                }
                if(gamepad2.x) {
                    robot.lift.liftState = LiftState.AUTO;
                    robot.lift.setTargetPos(Constants.sliderMedPos);
                    robotState = RobotState.LIFTING;
                }
                if(gamepad2.y) {
                    robot.lift.liftState = LiftState.AUTO;
                    robot.lift.setTargetPos(Constants.sliderHighPos);
                    robotState = RobotState.LIFTING;
                }
                break;
            case INTAKE:
                if(!gamepad2.a && lastGamepad2.a) {
                    robot.intake.wristState = WristState.RETRACT;
                    robotState = RobotState.RETRACT;
                }
                break;
            case LIFTING:
                if(robot.lift.getCurrentPos() > Constants.sliderWristClearPos) {
                    robotState = RobotState.OUTTAKE;
                    robot.intake.wristState = WristState.OUTTAKE;
                }
                if(gamepad2.a) {
                    robot.lift.liftState = LiftState.RETRACT;
                    robot.lift.resetSliderStallTimer();
                    robotState = RobotState.RETRACT;
                }
                break;
            case OUTTAKE:
                if(gamepad2.dpad_down) {
                    robot.intake.toggleWrist();
                }
                // manual lift control
                if(Math.abs(gamepad2.right_stick_y) > 0.1) {
                    robot.lift.liftState = LiftState.MANUAL;
                    robot.lift.power = -gamepad2.right_stick_y;
                } else {
                    robot.lift.liftState = LiftState.AUTO;
                }

                // slider presets
                if(gamepad2.b) {
                    robot.lift.liftState = LiftState.AUTO;
                    robot.lift.setTargetPos(Constants.sliderLowPos);
                }
                if(gamepad2.x) {
                    robot.lift.liftState = LiftState.AUTO;
                    robot.lift.setTargetPos(Constants.sliderMedPos);
                }
                if(gamepad2.y) {
                    robot.lift.liftState = LiftState.AUTO;
                    robot.lift.setTargetPos(Constants.sliderHighPos);
                }

                // retract
                if(gamepad2.a && robot.intake.wristState == WristState.RETRACT) {
                    robot.lift.liftState = LiftState.RETRACT;
                    robot.lift.resetSliderStallTimer();
                    robotState = RobotState.RETRACT;
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
                if(gamepad2.left_trigger > Constants.triggerSens) {
                    robot.intake.outtakeRollersPower = gamepad2.left_trigger;
                } else if(gamepad2.right_trigger > Constants.triggerSens) {
                    robot.intake.outtakeRollersPower = -gamepad2.right_trigger;
                } else {
                    robot.intake.outtakeRollersPower = 0;
                }

                // driving
                robot.drivetrain.setDrivePower(Constants.driveSpeedRetract);
                break;
            case INTAKE:
                // rollers power
                if(gamepad2.left_trigger > Constants.triggerSens) {
                    robot.intake.outtakeRollersPower = gamepad2.left_trigger;
                    robot.intake.intakeRollersPower = gamepad2.left_trigger;
                } else if(gamepad2.right_trigger > Constants.triggerSens) {
                    robot.intake.outtakeRollersPower = -gamepad2.right_trigger;
                    robot.intake.intakeRollersPower = -gamepad2.right_trigger;
                } else {
                    robot.intake.outtakeRollersPower = 0;
                    robot.intake.intakeRollersPower = 0;
                }

                robot.lift.setTargetPos(Constants.sliderIntakePos);
                robot.drivetrain.setDrivePower(Constants.driveSpeedIntake);
                break;
            case LIFTING:
                robot.drivetrain.setDrivePower(Constants.driveSpeedLifting);
                break;
            case OUTTAKE:
                robot.drivetrain.setDrivePower(Constants.driveSpeedOuttake);
                break;
        }


        // driving
        // resets heading offset (face forwards)
        if(gamepad1.right_stick_button) {
            robot.drivetrain.resetHeadingOffset();
        }
        robot.drivetrain.drive(Math.pow(gamepad1.left_stick_x, 3), Math.pow(-gamepad1.left_stick_y, 3), Math.pow(-gamepad1.right_stick_x, 3));

        robot.update();
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("robot state", robotState);
        robot.telemetry();
    }
}

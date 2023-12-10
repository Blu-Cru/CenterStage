package org.firstinspires.ftc.teamcode.BluCru.states;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Const;
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
                if(gamepad2.left_trigger > Constants.triggerSens && robot.lift.getCurrentPos() < Constants.sliderIntakeDelta) {
                    robotState = RobotState.INTAKE;
                    robot.intake.wristState = WristState.INTAKE;
                    robot.intake.intakeTimer.reset();
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
                if(!(gamepad2.left_trigger > Constants.triggerSens) && lastGamepad2.left_trigger > Constants.triggerSens) {
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
                    robot.lift.retractLift();
                    robot.lift.resetLiftStallTimer();
                    robotState = RobotState.RETRACT;
                }
                break;
            case OUTTAKE:
                if(gamepad2.dpad_down && !lastGamepad2.dpad_down) {
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
                    robot.lift.retractLift();
                    robotState = RobotState.RETRACT;
                }
                break;
        }

        // plane launcher
        if(gamepad2.dpad_right && !lastGamepad2.dpad_right) {
            robot.intake.togglePlane();
        }

        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);
    }

// moves robot based on state and gamepad input
    public void updateRobot(Gamepad gamepad1, Gamepad gamepad2) {
        switch (robotState) {
            case RETRACT:
                // driving
                robot.drivetrain.setDrivePower(Constants.driveSpeedRetract);
                break;
            case INTAKE:


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


        if(gamepad2.left_bumper) {
            robot.intake.outtakeRollersPower = Constants.outtakeRollersIntakePower;
        } else if(gamepad2.right_bumper) {
            robot.intake.outtakeRollersPower = Constants.outtakeRollersOuttakePower;
        } else if (gamepad2.left_trigger > Constants.triggerSens){
            robot.intake.outtakeRollersPower = Constants.outtakeRollersIntakePower;
        } else {
            robot.intake.outtakeRollersPower = 0;
        }

        if(gamepad2.left_trigger > Constants.triggerSens) {
            robot.intake.intakeRollersPower = Constants.intakeRollersIntakePower * gamepad2.left_trigger;
        } else if (gamepad2.right_trigger > Constants.triggerSens){
            robot.intake.intakeRollersPower = Constants.intakeRollersOuttakePower * gamepad2.right_trigger;
        } else {
            robot.intake.intakeRollersPower = 0;
        }

        if(Math.abs(gamepad2.left_stick_y) > 0.1) {
            robot.hanger.setPower(-gamepad2.left_stick_y);
        } else {
            robot.hanger.setPower(0);
        }

        // DRIVING

        double horz = Math.pow(gamepad1.left_stick_x, 3);
        double vert = Math.pow(-gamepad1.left_stick_y, 3);
        double rotate = Math.pow(-gamepad1.right_stick_x, 3);

        if(gamepad2.right_stick_button) {
            robot.lift.resetEncoder();
        }

        // driving
        // resets heading offset (face forwards)
        if(gamepad1.right_stick_button) {
            robot.drivetrain.resetIMU();
            gamepad1.rumble(100);
        }
        if(gamepad1.b) {
            robot.drivetrain.driveToHeading(horz, vert, Math.toRadians(90));
        } else if(gamepad1.x) {
            robot.drivetrain.driveToHeading(horz, vert, Math.toRadians(-90));
        } else {
            // otherwise, drive normally
            robot.drivetrain.drive(new Vector2d(horz, vert), rotate);
        }

        robot.update();
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("robot state", robotState);
        robot.telemetry(telemetry);
    }
}

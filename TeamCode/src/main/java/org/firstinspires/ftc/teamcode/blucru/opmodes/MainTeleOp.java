package org.firstinspires.ftc.teamcode.blucru.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blucru.Constants;
import org.firstinspires.ftc.teamcode.blucru.states.LiftState;
import org.firstinspires.ftc.teamcode.blucru.states.OuttakeState;
import org.firstinspires.ftc.teamcode.blucru.states.RobotState;
import org.firstinspires.ftc.teamcode.blucru.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.blucru.subsystems.Robot;


/*
Controls:

release everything: retracted state
a : release cone/intake
b : low height
x : medium height
y : high height
left joystick : strafe (field centric)
right joystick : turn

 */
@Config
@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends LinearOpMode {
    Robot robot;
    private RobotState robotState;

    private Gamepad lastGamepad1;
    private Gamepad lastGamepad2;
    ElapsedTime totalTimer;
    ElapsedTime outtakeTimer;
    double lastTime, deltaTime;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        while(opModeInInit()) {
            telemetry.addData("PICK UP UR CONTROLELRS", "");
            telemetry.update();
        }

        waitForStart();

        totalTimer = new ElapsedTime();

        while(opModeIsActive()) {
            lastTime = totalTimer.milliseconds();
            // updates states based on gamepad input
            updateStates();
            // updates robot based on states
            updateRobot();

            // loop time: current time - time at start of loop
            deltaTime = totalTimer.milliseconds() - lastTime;

            // data for feedback
            telemetry();
        }
    }

    public void initialize() {
        robotState = RobotState.RETRACT;
        lastGamepad1 = new Gamepad();
        lastGamepad2 = new Gamepad();
        robot = new Robot(telemetry, hardwareMap);
        robot.init();
    }

    public void read() {
        // DRIVING
        robot.drivetrain.setDrivePower(robotState, gamepad1);

        double horz = Math.pow(gamepad1.left_stick_x, 3);
        double vert = Math.pow(-gamepad1.left_stick_y, 3);
        double rotate = Math.pow(-gamepad1.right_stick_x, 3);

        // driving
        // resets heading offset (face forwards)
        if(gamepad1.right_stick_button) {
            robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
            gamepad1.rumble(100);
        }
        if(gamepad1.b) {
            robot.drivetrain.driveToHeading(horz, vert, Math.toRadians(180));
        } else if(gamepad1.x) {
            robot.drivetrain.driveToHeading(horz, vert, Math.toRadians(0));
        } else {
            // otherwise, drive normally
            robot.drivetrain.drive(horz, vert, rotate);
        }
        switch(robotState) {
            case RETRACT:
                if(gamepad2.b && !lastGamepad2.b) {
                    robotState = RobotState.OUTTAKE;
                    robot.outtake.targetHeight = Outtake.LOW_HEIGHT;
                }
                break;
            case INTAKE:
                break;
            case LIFTING:
                if(robot.outtake.lift.getCurrentPos() > Outtake.LIFT_WRIST_CLEAR_POS) {
                    robotState = RobotState.OUTTAKE;
                    robot.outtake.wristRetracted = false;
                }

                if(gamepad2.a) {
                    robot.outtake.outtakeState = OuttakeState.RETRACT;
                }
                break;
            case OUTTAKE:
                if(Math.abs(gamepad2.right_stick_y) > 0.1) {
                    robot.outtake.lift.liftState = LiftState.MANUAL;
                    robot.outtake.setManualSlidePower(-gamepad2.right_stick_y);
                } else {
                    robot.outtake.lift.liftState = LiftState.AUTO;
                }
                break;
        }
    }

    public void updateStates() {
        switch (robotState) {
            case RETRACT:
                if(gamepad2.left_trigger > Constants.triggerSens && robot.outtake.lift.getCurrentPos() < Constants.sliderIntakeDelta) {
                    robotState = RobotState.INTAKE;
//                    robot.intake.wristState = WristState.INTAKE;
                    robot.intake.intakeTimer.reset();
                }
                break;
            case INTAKE:
                if(!(gamepad2.left_trigger > Constants.triggerSens) && lastGamepad2.left_trigger > Constants.triggerSens) {
//                    robot.intake.wristState = WristState.RETRACT;
                    robotState = RobotState.RETRACT;
                }
                break;
            case OUTTAKE:
                if(gamepad2.dpad_down && !lastGamepad2.dpad_down) {
//                    robot.intake.toggleWrist();
                }
                // manual lift control
                if(Math.abs(gamepad2.right_stick_y) > 0.1) {
                    robot.outtake.lift.liftState = LiftState.MANUAL;
                    robot.outtake.lift.power = -gamepad2.right_stick_y;
                } else {
                    robot.outtake.lift.liftState = LiftState.AUTO;
                }

                // slider presets
                if(gamepad2.b) {
                    robot.outtake.lift.liftState = LiftState.AUTO;
                    robot.outtake.lift.setTargetPos(Constants.sliderLowPos);
                }
                if(gamepad2.x) {
                    robot.outtake.lift.liftState = LiftState.AUTO;
                    robot.outtake.lift.setTargetPos(Constants.sliderMedPos);
                }
                if(gamepad2.y) {
                    robot.outtake.lift.liftState = LiftState.AUTO;
                    robot.outtake.lift.setTargetPos(Constants.sliderHighPos);
                }

                // retract
//                if(gamepad2.a && robot.intake.wristState == WristState.RETRACT) {
//                    robot.lift.setTargetPos(0);
//                    robotState = RobotState.RETRACT;
//                }
                break;
        }

        // plane launcher
        if(gamepad2.dpad_right && !lastGamepad2.dpad_right) {
//            robot.intake.togglePlane();
        }

        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);
    }

    public void updateRobot() {
        switch (robotState) {
            case RETRACT:
                // driving
                robot.drivetrain.setDrivePower(Constants.driveSpeedRetract);
                break;
            case INTAKE:
                robot.outtake.lift.setTargetPos(Constants.sliderIntakePos);
                robot.drivetrain.setDrivePower(Constants.driveSpeedIntake);
                break;
            case OUTTAKE:
                robot.drivetrain.setDrivePower(Constants.driveSpeedOuttake);
                break;
        }


//        if(gamepad2.left_bumper) {
//            robot.intake.outtakeRollersPower = Constants.outtakeRollersIntakePower;
//        } else if(gamepad2.right_bumper) {
//            robot.intake.outtakeRollersPower = Constants.outtakeRollersOuttakePower;
//        } else if (gamepad2.left_trigger > Constants.triggerSens){
//            robot.intake.outtakeRollersPower = Constants.outtakeRollersIntakePower;
//        } else {
//            robot.intake.outtakeRollersPower = 0;
//        }

//        if(gamepad2.left_trigger > Constants.triggerSens) {
//            robot.intake.intakeRollersPower = Constants.intakeRollersIntakePower * gamepad2.left_trigger;
//        } else if (gamepad2.right_trigger > Constants.triggerSens){
//            robot.intake.intakeRollersPower = Constants.intakeRollersOuttakePower * gamepad2.right_trigger;
//        } else {
//            robot.intake.intakeRollersPower = 0;
//        }

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
            robot.outtake.lift.resetEncoder();
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
            robot.drivetrain.drive(horz, vert, rotate);
        }

        robot.write();
    }

    public void telemetry() {
        telemetry.addData("robot state", robotState);
        telemetry.addData("loop time", deltaTime);
        robot.telemetry(telemetry);
        telemetry.update();
    }
}

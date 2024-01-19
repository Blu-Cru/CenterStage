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
    public static double OUTTAKE_DELAY_SECONDS = 1;

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


        while(opModeIsActive()) {
            lastTime = totalTimer.milliseconds();
            // updates states based on gamepad input
            read();

            // loop time: current time - time at start of loop
            deltaTime = totalTimer.milliseconds() - lastTime;

            // data for feedback
            write();
        }
    }

    public void initialize() {
        robotState = RobotState.RETRACT;
        lastGamepad1 = new Gamepad();
        lastGamepad2 = new Gamepad();
        robot = new Robot(telemetry, hardwareMap);

        totalTimer = new ElapsedTime();
        outtakeTimer = new ElapsedTime();

        robot.init();
    }

    public void read() {
        // DRIVING
        robot.drivetrain.setDrivePower(robotState, gamepad1);

        double horz = Math.pow(gamepad1.left_stick_x, 3);
        double vert = Math.pow(-gamepad1.left_stick_y, 3);
        double rotate = Math.pow(-gamepad1.right_stick_x, 3);

        // resets heading offset (face forwards)
        if(gamepad1.right_stick_button) {
            robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
            gamepad1.rumble(100);
        }
        if(gamepad1.b) {
            robot.drivetrain.driveToHeading(horz, vert, Math.toRadians(180));
        } else if(gamepad1.x) {
            robot.drivetrain.driveToHeading(horz, vert, 0);
        } else {
            // otherwise, drive normally
            robot.drivetrain.drive(horz, vert, rotate);
        }

        if(gamepad2.left_trigger > 0.1) {
            robot.intake.setIntakePower(gamepad2.left_trigger);
        } else if(gamepad2.right_trigger > 0.1) {
            robot.intake.setIntakePower(-gamepad2.right_trigger);
        } else {
            robot.intake.setIntakePower(0);
        }

        if(gamepad2.a) {
            robot.intake.downIntakeWrist();
        } else {
            robot.intake.retractIntakeWrist();
        }

        switch(robotState) {
            case RETRACT:
                robot.outtake.outtakeState = OuttakeState.RETRACT;

                if(gamepad2.a && robot.outtake.liftIntakeReady()) {
                    robot.intake.downIntakeWrist();
                } else {
                    robot.intake.retractIntakeWrist();
                }

                if(gamepad2.b && !lastGamepad2.b) {
                    robotState = RobotState.LIFTING;
                    robot.outtake.targetHeight = Outtake.LOW_HEIGHT;
                }
                if(gamepad2.x && !lastGamepad2.x) {
                    robotState = RobotState.LIFTING;
                    robot.outtake.targetHeight = Outtake.MED_HEIGHT;
                }
                if(gamepad2.y && !lastGamepad2.y) {
                    robotState = RobotState.LIFTING;
                    robot.outtake.targetHeight = Outtake.HIGH_HEIGHT;
                }
                break;
            case LIFTING:
                robot.outtake.outtakeState = OuttakeState.OUTTAKE;
                if(robot.outtake.lift.getCurrentPos() > Outtake.LIFT_WRIST_CLEAR_POS) {
                    robot.outtake.wristRetracted = false;
                    robotState = RobotState.OUTTAKE;
                    outtakeTimer.reset();
                }

                if(gamepad2.b && !lastGamepad2.b) {
                    robotState = RobotState.LIFTING;
                    robot.outtake.targetHeight = Outtake.LOW_HEIGHT;
                }
                if(gamepad2.x && !lastGamepad2.x) {
                    robotState = RobotState.LIFTING;
                    robot.outtake.targetHeight = Outtake.MED_HEIGHT;
                }
                if(gamepad2.y && !lastGamepad2.y) {
                    robotState = RobotState.LIFTING;
                    robot.outtake.targetHeight = Outtake.HIGH_HEIGHT;
                }

                if(gamepad2.a) {
                    robot.outtake.outtakeState = OuttakeState.RETRACT;
                    robot.outtake.lift.setMotionProfileTargetPos(0);
                }
                break;
            case OUTTAKE:
                robot.outtake.outtakeState = OuttakeState.OUTTAKE;

                if(outtakeTimer.seconds() > OUTTAKE_DELAY_SECONDS) {
                    if(gamepad2.left_trigger > 0.1) {
                        robot.outtake.setTurretAngle(-gamepad2.left_trigger * 90 + 270);
                    } else if (gamepad2.right_trigger > 0.1) {
                        robot.outtake.setTurretAngle(gamepad2.right_trigger * 90 + 270);
                    }
                } else {
                    if(gamepad2.dpad_down) {
                        robot.outtake.toggleWrist();
                    }
                    robot.outtake.setTurretAngle(270);
                }

                if(gamepad2.b && !lastGamepad2.b) {
                    robot.outtake.targetHeight = Outtake.LOW_HEIGHT;
                }
                if(gamepad2.x && !lastGamepad2.x) {
                    robot.outtake.targetHeight = Outtake.MED_HEIGHT;
                }
                if(gamepad2.y && !lastGamepad2.y) {
                    robot.outtake.targetHeight = Outtake.HIGH_HEIGHT;
                }

                if(gamepad2.a && !lastGamepad2.a && robot.outtake.wristRetracted) {
                    robotState = RobotState.RETRACT;
                    robot.outtake.lift.setMotionProfileTargetPos(0);
                }
                break;
        }

        if(Math.abs(gamepad2.right_stick_y) > 0.1) {
            robot.outtake.lift.liftState = LiftState.MANUAL;
            robot.outtake.setManualSlidePower(-gamepad2.right_stick_y);
        } else {
            if(!(Math.abs(lastGamepad2.right_stick_y) > 0.1)) {
                robot.outtake.updateTargetHeight();
            }
            robot.outtake.lift.liftState = LiftState.AUTO;
        }
    }

    public void write() {
        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);

        robot.write();

        telemetry.addData("robot state", robotState);
        telemetry.addData("loop time", deltaTime);
        robot.telemetry(telemetry);
        telemetry.update();
    }
}

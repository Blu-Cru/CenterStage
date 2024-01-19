package org.firstinspires.ftc.teamcode.blucru.testopmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blucru.states.LiftState;
import org.firstinspires.ftc.teamcode.blucru.states.OuttakeState;
import org.firstinspires.ftc.teamcode.blucru.states.RobotState;
import org.firstinspires.ftc.teamcode.blucru.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.subsystems.Outtake;

public class OuttakeTest extends LinearOpMode {
    Outtake outtake;
    Drivetrain drivetrain;
    RobotState robotState;
    Gamepad lastGamepad1;
    Gamepad lastGamepad2;
    ElapsedTime outtakeTimer;

    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        while(opModeIsActive()) {
            read();
        }
    }

    public void initialize() {
        outtake = new Outtake(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        robotState = RobotState.RETRACT;

        lastGamepad1 = new Gamepad();
        lastGamepad2 = new Gamepad();

        outtakeTimer = new ElapsedTime();

        outtake.init();
        drivetrain.init();
    }

    public void read() {
        drivetrain.setDrivePower(robotState, gamepad1);

        double horz = Math.pow(gamepad1.left_stick_x, 3);
        double vert = Math.pow(-gamepad1.left_stick_y, 3);
        double rotate = Math.pow(-gamepad1.right_stick_x, 3);

        // resets heading offset (face forwards)
        if(gamepad1.right_stick_button) {
            drivetrain.resetHeading();
            gamepad1.rumble(100);
        }
        if(gamepad1.b) {
            drivetrain.driveToHeading(horz, vert, Math.toRadians(180));
        } else if(gamepad1.x) {
            drivetrain.driveToHeading(horz, vert, 0);
        } else {
            // otherwise, drive normally
            drivetrain.drive(horz, vert, rotate);
        }

        if(Math.abs(gamepad2.right_stick_y) > 0.1) {
            outtake.lift.liftState = LiftState.MANUAL;
            outtake.setManualSlidePower(-gamepad2.right_stick_y);
        } else {
            if(!(Math.abs(lastGamepad2.right_stick_y) > 0.1)) {
                outtake.updateTargetHeight();
            }
            outtake.lift.liftState = LiftState.AUTO;
        }

        switch (robotState) {
            case RETRACT:
                outtake.outtakeState = OuttakeState.RETRACT;

                if(gamepad2.b && !lastGamepad2.b) {
                    robotState = RobotState.LIFTING;
                    outtake.targetHeight = Outtake.LOW_HEIGHT;
                }
                if(gamepad2.x && !lastGamepad2.x) {
                    robotState = RobotState.LIFTING;
                    outtake.targetHeight = Outtake.MED_HEIGHT;
                }
                if(gamepad2.y && !lastGamepad2.y) {
                    robotState = RobotState.LIFTING;
                    outtake.targetHeight = Outtake.HIGH_HEIGHT;
                }
                break;
            case LIFTING:
                outtake.outtakeState = OuttakeState.OUTTAKE;
                if(outtake.lift.getCurrentPos() > Outtake.LIFT_WRIST_CLEAR_POS) {
                    outtake.wristRetracted = false;
                    robotState = RobotState.OUTTAKE;
                    outtakeTimer.reset();
                }

                if(gamepad2.b && !lastGamepad2.b) {
                    robotState = RobotState.LIFTING;
                    outtake.targetHeight = Outtake.LOW_HEIGHT;
                }
                if(gamepad2.x && !lastGamepad2.x) {
                    robotState = RobotState.LIFTING;
                    outtake.targetHeight = Outtake.MED_HEIGHT;
                }
                if(gamepad2.y && !lastGamepad2.y) {
                    robotState = RobotState.LIFTING;
                    outtake.targetHeight = Outtake.HIGH_HEIGHT;
                }

                if(gamepad2.a) {
                    outtake.outtakeState = OuttakeState.RETRACT;
                    outtake.lift.setMotionProfileTargetPos(0);
                }
                break;
            case OUTTAKE:
                outtake.outtakeState = OuttakeState.OUTTAKE;

                if(outtakeTimer.seconds() > 1 && !outtake.wristRetracted) {
                    if(gamepad2.left_trigger > 0.1) {
                        outtake.setTurretAngle(-gamepad2.left_trigger * 90 + 270);
                    } else if (gamepad2.right_trigger > 0.1) {
                        outtake.setTurretAngle(gamepad2.right_trigger * 90 + 270);
                    }
                } else {
                    outtake.setTurretAngle(270);
                }

                if(outtake.getTurretAngle() == 270) {
                    if(gamepad2.dpad_down && !lastGamepad2.dpad_down) {
                        outtake.toggleWrist();
                    }
                }

                if(gamepad2.b && !lastGamepad2.b) {
                    outtake.targetHeight = Outtake.LOW_HEIGHT;
                }
                if(gamepad2.x && !lastGamepad2.x) {
                    outtake.targetHeight = Outtake.MED_HEIGHT;
                }
                if(gamepad2.y && !lastGamepad2.y) {
                    outtake.targetHeight = Outtake.HIGH_HEIGHT;
                }

                if(gamepad2.a && !lastGamepad2.a && outtake.wristRetracted) {
                    outtake.outtakeState = OuttakeState.RETRACT;
                    robotState = RobotState.RETRACT;
                    outtake.lift.setMotionProfileTargetPos(0);
                }
                break;
        }
    }
}

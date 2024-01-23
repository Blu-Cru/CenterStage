package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blucru.common.states.OuttakeState;
import org.firstinspires.ftc.teamcode.blucru.common.states.RobotState;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

@TeleOp(name = "outtake test", group = "TeleOp")
public class OuttakeTest extends LinearOpMode {
    Robot robot;
    Drivetrain drivetrain;
    Outtake outtake;
    RobotState robotState;
    Gamepad lastGamepad1;
    Gamepad lastGamepad2;
    ElapsedTime outtakeTimer;
    double lastTime = 0;
    double dt = 0;

    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        drivetrain = robot.addDrivetrain();
        outtake = robot.addOuttake();

        initialize();

        waitForStart();

        while(opModeIsActive()) {
            robot.read();

            dt = System.currentTimeMillis() - lastTime;
            lastTime = System.currentTimeMillis();

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

            if(gamepad2.left_bumper) {
                outtake.unlock();
            } else {
                outtake.lock();
            }

//        if(Math.abs(gamepad2.right_stick_y) > 0.1) {
//            outtake.lift.liftState = LiftState.MANUAL;
//            outtake.setManualSlidePower(-gamepad2.right_stick_y);
//        } else {
//            if(!(Math.abs(gamepad2.right_stick_y) > 0.1) && Math.abs(lastGamepad2.right_stick_y) > 0.1) {
//                outtake.updateTargetHeight();
//            }
//            outtake.lift.liftState = LiftState.AUTO;
//        }

            switch (robotState) {
                case RETRACT:
                    outtake.outtakeState = OuttakeState.RETRACT;

                    if(gamepad2.b) {
                        robotState = RobotState.LIFTING;
                        outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                    }
                    if(gamepad2.x) {
                        robotState = RobotState.LIFTING;
                        outtake.setTargetHeight(Outtake.MED_HEIGHT);
                    }
                    if(gamepad2.y) {
                        robotState = RobotState.LIFTING;
                        outtake.setTargetHeight(Outtake.HIGH_HEIGHT);
                    }
                    break;
                case LIFTING:
                    outtake.outtakeState = OuttakeState.OUTTAKE;
                    if(outtake.lift.getCurrentPos() > Outtake.LIFT_WRIST_CLEAR_POS) {
                        outtake.wristRetracted = false;
                        robotState = RobotState.OUTTAKE;
                        outtakeTimer.reset();
                    }

                    if(gamepad2.b) {
                        robotState = RobotState.LIFTING;
                        outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                    }
                    if(gamepad2.x) {
                        robotState = RobotState.LIFTING;
                        outtake.setTargetHeight(Outtake.MED_HEIGHT);
                    }
                    if(gamepad2.y) {
                        robotState = RobotState.LIFTING;
                        outtake.setTargetHeight(Outtake.HIGH_HEIGHT);
                    }

                    if(gamepad2.a) {
                        outtake.outtakeState = OuttakeState.RETRACT;
                        outtake.lift.setTargetPos(0);
                    }
                    break;
                case OUTTAKE:
                    outtake.outtakeState = OuttakeState.OUTTAKE;

                    if(outtakeTimer.seconds() > 1) {
                        if(!outtake.wristRetracted) {
                            if(gamepad2.left_trigger > 0.1) {
                                outtake.setTurretAngle(-60 + 270);
                            } else if (gamepad2.right_trigger > 0.1) {
                                outtake.setTurretAngle(60 + 270);
                            } else {
                                outtake.setTurretAngle(270);
                            }
                        } else {
                            outtake.setTurretAngle(270);
                        }
                    } else {
                        outtake.setTurretAngle(270);
                    }

                    if(Math.abs(outtake.getTurretAngle() - 270) < 10) {
                        if(gamepad2.dpad_up) {
                            outtake.extendWrist();
                        }
                        if(gamepad2.dpad_down) {
                            outtake.retractWrist();
                        }
                    } else {
                        outtake.extendWrist();
                    }

                    if(gamepad2.b) {
                        outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                    }
                    if(gamepad2.x) {
                        outtake.setTargetHeight(Outtake.MED_HEIGHT);
                    }
                    if(gamepad2.y) {
                        outtake.setTargetHeight(Outtake.HIGH_HEIGHT);
                    }

                    if(gamepad2.a && outtake.wristRetracted) {
                        outtake.outtakeState = OuttakeState.RETRACT;
                        robotState = RobotState.RETRACT;
                        outtake.lift.setTargetPos(0);
                    }
                    break;
            }

            robot.write();

            lastGamepad1.copy(gamepad1);
            lastGamepad2.copy(gamepad2);
            telemetry.addData("loop time", dt);
            telemetry.addData("robot state", robotState);
            robot.telemetry(telemetry);
            telemetry.update();
        }
    }

    public void initialize() {
        robotState = RobotState.RETRACT;

        lastGamepad1 = new Gamepad();
        lastGamepad2 = new Gamepad();

        outtakeTimer = new ElapsedTime();
    }
}

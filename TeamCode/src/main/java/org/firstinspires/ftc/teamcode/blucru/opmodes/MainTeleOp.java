package org.firstinspires.ftc.teamcode.blucru.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blucru.states.Initialization;
import org.firstinspires.ftc.teamcode.blucru.states.LiftState;
import org.firstinspires.ftc.teamcode.blucru.states.OuttakeState;
import org.firstinspires.ftc.teamcode.blucru.states.RobotState;
import org.firstinspires.ftc.teamcode.blucru.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.subsystems.Lift;
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
@TeleOp(name = "Main TeleOp", group = "1")
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
            deltaTime = totalTimer.milliseconds() - lastTime;
            lastTime = totalTimer.milliseconds();
            // updates states based on gamepad input
            read();

            // loop time: current time - time at start of loop

            // data for feedback
            write();
        }
    }

    public void initialize() {
        robotState = RobotState.RETRACT;
        lastGamepad1 = new Gamepad();
        lastGamepad2 = new Gamepad();
        robot = new Robot(hardwareMap);

        totalTimer = new ElapsedTime();
        outtakeTimer = new ElapsedTime();

        robot.init();

        // set initial pose from auto
        robot.drivetrain.setPoseEstimate(Initialization.POSE);
    }

    public void read() {
        robot.clearBulkCache();

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
            if(gamepad1.left_bumper) {
                robot.drivetrain.driveToDistanceToHeading(horz, vert, Drivetrain.OUTTAKE_DISTANCE, Math.toRadians(180));
            } else {
                robot.drivetrain.driveToHeading(horz, vert, Math.toRadians(180));
            }
        } else if(gamepad1.x) {
            if(gamepad1.left_bumper) {
                robot.drivetrain.driveToDistanceToHeading(horz, vert, Drivetrain.OUTTAKE_DISTANCE, 0);
            } else {
                robot.drivetrain.driveToHeading(horz, vert, 0);
            }
        } else {
            robot.drivetrain.drive(horz, vert, rotate);
        }

        // INTAKE
        if(gamepad1.left_trigger > 0.1) {
            robot.intake.setIntakePower(gamepad1.left_trigger);
            robot.outtake.unlock();
        } else if(gamepad1.right_trigger > 0.1) {
            robot.intake.setIntakePower(-gamepad1.right_trigger);
            robot.outtake.lock();
        } else {
            robot.intake.setIntakePower(0);
            if(gamepad2.left_bumper) {
                robot.outtake.lockBack();
            } else if(gamepad2.right_bumper) {
                robot.outtake.unlock();
            } else {
                robot.outtake.lock();
            }
        }

        // toggle intake wrist
        if(gamepad2.a && robot.outtake.liftIntakeReady()) {
            robot.intake.downIntakeWrist();
        } else {
            robot.intake.retractIntakeWrist();
        }

        switch(robotState) {
            case RETRACT:
                robot.outtake.outtakeState = OuttakeState.RETRACT;

                if(gamepad2.b) {
                    robotState = RobotState.LIFTING;
                    robot.outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                }
                if(gamepad2.x) {
                    robotState = RobotState.LIFTING;
                    robot.outtake.setTargetHeight(Outtake.MED_HEIGHT);
                }
                if(gamepad2.y) {
                    robotState = RobotState.LIFTING;
                    robot.outtake.setTargetHeight(Outtake.HIGH_HEIGHT);
                }
                break;
            case LIFTING:
                robot.outtake.outtakeState = OuttakeState.OUTTAKE;
                if(robot.outtake.lift.getCurrentPos() > Outtake.LIFT_WRIST_CLEAR_POS) {
                    robot.outtake.wristRetracted = false;
                    robotState = RobotState.OUTTAKE;
                    outtakeTimer.reset();
                }

                if(gamepad2.b) {
                    robotState = RobotState.LIFTING;
                    robot.outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                }
                if(gamepad2.x) {
                    robotState = RobotState.LIFTING;
                    robot.outtake.setTargetHeight(Outtake.MED_HEIGHT);
                }
                if(gamepad2.y) {
                    robotState = RobotState.LIFTING;
                    robot.outtake.setTargetHeight(Outtake.HIGH_HEIGHT);
                }

                if(gamepad2.a) {
                    robotState = RobotState.RETRACT;
                    robot.outtake.outtakeState = OuttakeState.RETRACT;
                    robot.outtake.lift.setTargetPos(0);
                }
                break;
            case OUTTAKE:
                robot.outtake.outtakeState = OuttakeState.OUTTAKE;

                if(outtakeTimer.seconds() > OUTTAKE_DELAY_SECONDS) {
                    if(!robot.outtake.wristRetracted) {
                        if (gamepad2.left_trigger > 0.1) {
                            robot.outtake.setTurretAngle(-gamepad2.left_trigger * 60 + 270);
                        } else if (gamepad2.right_trigger > 0.1) {
                            robot.outtake.setTurretAngle(gamepad2.right_trigger * 60 + 270);
                        } else {
                            robot.outtake.setTurretAngle(270);
                        }
                    } else {
                        robot.outtake.setTurretAngle(270);
                    }
                } else {
                    robot.outtake.setTurretAngle(270);
                }

                if(Math.abs(robot.outtake.getTurretAngle() - 270) < 10) {
                    if(gamepad2.dpad_up) {
                        robot.outtake.extendWrist();
                    }
                    if(gamepad2.dpad_down) {
                        robot.outtake.retractWrist();
                    }
                } else {
                    robot.outtake.extendWrist();
                }

                if(gamepad2.b) {
                    robot.outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                }
                if(gamepad2.x) {
                    robot.outtake.setTargetHeight(Outtake.MED_HEIGHT);
                }
                if(gamepad2.y) {
                    robot.outtake.setTargetHeight(Outtake.HIGH_HEIGHT);
                }

                if(gamepad2.a && robot.outtake.wristRetracted) {
                    robotState = RobotState.RETRACT;
                    robot.outtake.outtakeState = OuttakeState.RETRACT;
                    robot.outtake.lift.setTargetPos(0);
                }
                break;
        }

        // MANUAL SLIDE
//        if(Math.abs(gamepad2.right_stick_y) > 0.1) {
//            robot.outtake.lift.liftState = LiftState.MANUAL;
//            robot.outtake.setManualSlidePower(-gamepad2.right_stick_y + Lift.kF);
//        } else {
//            if(!(Math.abs(lastGamepad2.right_stick_y) > 0.1)) {
//                robot.outtake.updateTargetHeight();
//            }
//            robot.outtake.lift.liftState = LiftState.AUTO;
//        }

        // MANUAL HANG
        if(Math.abs(gamepad2.left_stick_y) > 0.1) {
            robot.hanger.setPower(-gamepad2.left_stick_y);
        } else {
            robot.hanger.setPower(0);
        }

        robot.read();
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

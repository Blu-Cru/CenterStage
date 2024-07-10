package org.firstinspires.ftc.teamcode.blucru.opmode.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.states.RobotState;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.blucru.opmode.KLinearOpMode;

@Disabled
@TeleOp(name = "Solo", group = "2")
// this doesnt wrok
public class Solo extends KLinearOpMode {
    public static double OUTTAKE_TURN_TURRET_DELAY = 300;
    public static double REVERSE_INTAKE_TIME = 1000;
    public static double RETRACT_WRIST_DELAY = 250;
    public static double FULL_RETRACT_DELAY_AFTER_WRIST = 100;

    RobotState robotState;

    double scoringHeading;

    // timer variables
    double stopIntakeTime = -10000; // init at -10000 ms to prevent immediate outtake
    double outtakeTime = 0;
    double retractTime = 0;
    double retractWristTime = 0;

    // gamepad variables
    double lastIntakePower;
    boolean lastA1;
    boolean lastDown1;

    public void periodic() {
        switch (robotState) {
            case RETRACT:
                // drop down and intake
                if(outtake.liftIntakeReady()){
                    if(gamepad1.a && gamepad1.left_bumper) {
                        intake.dropToStack(3);
                        intake.setPower(1);
                        outtake.unlock();
                    } else if(gamepad1.a) {
                        intake.dropToGround();
                        intake.setPower(1);
                        outtake.unlock();
                    } else if(gamepad1.left_trigger > 0.3) {
                        intake.retractIntakeWrist();
                        intake.setPower(gamepad1.left_trigger);
                        outtake.unlock();
                    } else if(gamepad1.right_trigger > 0.3) {
                        intake.retractIntakeWrist();
                        intake.setPower(-gamepad1.right_trigger);
                        outtake.lock();
                    } else if(timeSince(stopIntakeTime) < REVERSE_INTAKE_TIME) {
                        intake.retractIntakeWrist();
                        intake.setPower(-1);
                        outtake.lock();
                    } else {
                        intake.retractIntakeWrist();
                        intake.setPower(0);
                        outtake.lock();
                    }
                }

                // if intake just stopped, start timer
                if(lastIntakePower > 0.1 && !(intake.getIntakePower() > 0.1)) stopIntakeTime = currentTime();
                lastIntakePower = intake.getIntakePower();

                if(gamepad1.x) {
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                }
                if(gamepad1.y) {
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.MED_HEIGHT);
                }
                if(gamepad1.b) {
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.HIGH_HEIGHT);
                }
                break;
            case LIFTING:
                // stop intake
                intake.retractIntakeWrist();
                intake.setPower(0);

                if(outtake.liftWristClear()) {
//                    outtake.wristRetracted = false;
                    robotState = RobotState.OUTTAKING;
                    outtakeTime = currentTime();
                }

                if(gamepad1.x) outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                if(gamepad1.y) outtake.setTargetHeight(Outtake.MED_HEIGHT);
                if(gamepad1.b) outtake.setTargetHeight(Outtake.HIGH_HEIGHT);

// reverse intake
                if(gamepad1.right_trigger > 0.3 && gamepad1.left_trigger > 0.3) intake.setPower(-(gamepad1.right_trigger + gamepad1.left_trigger)/2);
                else intake.setPower(0);

                if(gamepad1.a && !lastA1) {
                    robotState = RobotState.RETRACT;
                    outtake.retractLift();
                }
                lastA1 = gamepad1.a;

                break;
            case OUTTAKING:
                // TURRET CONTROL
                if(timeSince(outtakeTime) > OUTTAKE_TURN_TURRET_DELAY) {
                    if (gamepad1.left_trigger > 0.1) outtake.setTurretAngle(-gamepad1.left_trigger * 60 + 270);
                    else if (gamepad1.right_trigger > 0.1) outtake.setTurretAngle(gamepad1.right_trigger * 60 + 270);
                    else outtake.centerTurret();
                } else outtake.centerTurret();

                // retract wrist
                if(outtake.turret.isCentered() && gamepad1.dpad_down && !lastDown1) {
                    outtake.wristRetract();
                    robotState = RobotState.OUTTAKE_WRIST_RETRACTED;
                }
                lastDown1 = gamepad1.dpad_down;

                // Change height
                if(gamepad1.x) outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                if(gamepad1.y) outtake.setTargetHeight(Outtake.MED_HEIGHT);
                if(gamepad1.b) outtake.setTargetHeight(Outtake.HIGH_HEIGHT);

// reverse intake
                if(gamepad1.right_trigger > 0.3 && gamepad1.left_trigger > 0.3) intake.setPower(-(gamepad1.right_trigger + gamepad1.left_trigger)/2);
                else intake.setPower(0);

// lock/unlock for depositing
                if(gamepad1.left_bumper) outtake.unlockFrontLockBack();
                else if(gamepad1.right_bumper) outtake.unlock();
                else outtake.lock();

                // retract
                if(gamepad1.a && !lastA1) {
                    retractTime = currentTime();
                    outtake.centerTurret();
                    outtake.incrementTargetHeight(1);
                    robotState = RobotState.RETRACTING;
                }
                lastA1 = gamepad1.a;
                break;
            case OUTTAKE_WRIST_RETRACTED:
                // stop intake
                intake.retractIntakeWrist();
                intake.setPower(0);

                // retract
                if(gamepad1.a && !lastA1) {
                    robotState = RobotState.RETRACT;
                    outtake.retractLift();
                }

                // extend wrist
                if(gamepad1.dpad_down && !lastDown1) {
                    outtake.wristExtend();
                    robotState = RobotState.OUTTAKING;
                    outtakeTime = currentTime();
                }
                lastDown1 = gamepad1.dpad_down;

                // Change height
                if(gamepad1.x) {
                    outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                }
                if(gamepad1.y) {
                    outtake.setTargetHeight(Outtake.MED_HEIGHT);
                }
                if(gamepad1.b) {
                    outtake.setTargetHeight(Outtake.HIGH_HEIGHT);
                }
                break;
            case RETRACTING:
                // stop intake
                intake.retractIntakeWrist();
                intake.setPower(0);

                // retract wrist
                if(timeSince(retractTime) > RETRACT_WRIST_DELAY && outtake.lift.getAbsPosError() < 30) {
                    outtake.wristRetract();
                    retractWristTime = currentTime();
                }

                // fully retract
                if(timeSince(retractWristTime) > FULL_RETRACT_DELAY_AFTER_WRIST && outtake.lift.getAbsPosError() < 30) {
                    outtake.retractLift();
                    robotState = RobotState.RETRACT;
                }
                break;
        }

// DRIVING

        drivetrain.setTeleDrivePower(robotState, gamepad1);

        double horz = gamepad1.left_stick_x;
        double vert = -gamepad1.left_stick_y;
        double rotate = -gamepad1.right_stick_x;

        if (gamepad1.left_stick_button) drivetrain.driveToHeadingScaled(horz, vert, scoringHeading);
        else drivetrain.teleOpDrive(horz, vert, rotate);

        if(gamepad1.right_stick_button) {
            drivetrain.resetHeading(scoringHeading);
            gamepad1.rumble(100);
        }
    }

    public void initialize() {
        addDrivetrain(true);
        addIntake();
        addOuttake();
        addHanger();
        addPlane();

        robotState = RobotState.RETRACT;
        scoringHeading = alliance == Alliance.RED ? Math.toRadians(180) : 0;
    }

    public void telemetry() {
        telemetry.addData("robot state", robotState);
    }
}

package org.firstinspires.ftc.teamcode.blucru.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.states.RobotState;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.blucru.common.util.BCLinearOpMode;

@TeleOp(name = "Solo", group = "2")
public class Solo extends BCLinearOpMode {
    public static double OUTTAKE_DELAY = 300;
    public static double REVERSE_INTAKE_TIME = 1000;
    public static double RETRACT_DELAY = 350;

    RobotState robotState;

    double boardHeading;
    boolean retractRequested = false;

    // timer variables
    double stopIntakeTime = -10000; // init at -10000 ms to prevent immediate outtake
    double outtakeTime = 0;
    double retractTime = 0;

    // gamepad variables
    double lastLT = 0;
    boolean lastA1;
    boolean lastDown1;

    public void periodic() {
        switch (robotState) {
            case RETRACT:
                outtake.outtaking = false;
                // drop down
                if(gamepad1.a && gamepad1.left_bumper) intake.dropToStack(3);
                else if(gamepad1.a) intake.dropToGround();
                else intake.retractIntakeWrist();

                // intake/outtake
                if(gamepad1.left_trigger > 0.3) {
                    intake.setIntakePower(gamepad1.left_trigger);
                    outtake.unlock();
                } else if(gamepad1.right_trigger > 0.3) {
                    intake.setIntakePower(-gamepad1.right_trigger);
                    outtake.lock();
                } else if(timeSince(stopIntakeTime) < REVERSE_INTAKE_TIME) {
                    intake.setIntakePower(-1);
                    outtake.lock();
                } else {
                    intake.setIntakePower(0);
                    outtake.lock();
                }

                // if LT was released, start timer
                if(lastLT > 0.3 && !(gamepad1.left_trigger > 0.3))  stopIntakeTime = currentTime();
                lastLT = gamepad1.left_trigger;

                if(gamepad1.x) {
                    outtake.outtaking = true;
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                }
                if(gamepad1.y) {
                    outtake.outtaking = true;
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.MED_HEIGHT);
                }
                if(gamepad1.b) {
                    outtake.outtaking = true;
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.HIGH_HEIGHT);
                }

                break;
            case LIFTING:
                outtake.outtaking = true;

                if(outtake.lift.getCurrentPos() > Outtake.LIFT_WRIST_CLEAR_POS) {
                    outtake.wristRetracted = false;
                    robotState = RobotState.OUTTAKE_WRIST_UP;
                    outtakeTime = currentTime();
                }

                if(gamepad1.x) outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                if(gamepad1.y) outtake.setTargetHeight(Outtake.MED_HEIGHT);
                if(gamepad1.b) outtake.setTargetHeight(Outtake.HIGH_HEIGHT);

// reverse intake
                if(gamepad1.right_trigger > 0.3 && gamepad1.left_trigger > 0.3) intake.setIntakePower(-(gamepad1.right_trigger + gamepad1.left_trigger)/2);
                else intake.setIntakePower(0);

                if(gamepad1.a && !lastA1) {
                    robotState = RobotState.RETRACT;
                    retractRequested = false;
                    outtake.outtaking = false;
                    outtake.lift.setMotionProfileTargetPos(0);
                }
                lastA1 = gamepad1.a;

                break;
            case OUTTAKE_WRIST_UP:
                outtake.outtaking = true;

                // TURRET CONTROL
                if(timeSince(outtakeTime) > OUTTAKE_DELAY && !outtake.wristRetracted) {
                    if (gamepad1.left_trigger > 0.1) outtake.setTurretAngle(-gamepad1.left_trigger * 60 + 270);
                    else if (gamepad1.right_trigger > 0.1) outtake.setTurretAngle(gamepad1.right_trigger * 60 + 270);
                    else outtake.centerTurret();
                } else outtake.centerTurret();

                // retract wrist
                if(outtake.turret.isCentered() && gamepad1.dpad_down && !lastDown1) {
                    outtake.retractWrist();
                    robotState = RobotState.OUTTAKE_WRIST_RETRACTED;
                }
                lastDown1 = gamepad1.dpad_down;

                // Change height
                if(gamepad1.x) outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                if(gamepad1.y) outtake.setTargetHeight(Outtake.MED_HEIGHT);
                if(gamepad1.b) outtake.setTargetHeight(Outtake.HIGH_HEIGHT);

// reverse intake
                if(gamepad1.right_trigger > 0.3 && gamepad1.left_trigger > 0.3) intake.setIntakePower(-(gamepad1.right_trigger + gamepad1.left_trigger)/2);
                else intake.setIntakePower(0);

// lock/unlock for depositing
                if(gamepad1.left_bumper) outtake.unlockFrontLockBack();
                else if(gamepad1.right_bumper) outtake.unlock();
                else outtake.lock();

                // retract
                if(gamepad1.a && !lastA1 && outtake.turret.isCentered()) {
                    retractRequested = true;
                    retractTime = currentTime();
                    outtake.retractWrist();
                    robotState = RobotState.OUTTAKE_WRIST_RETRACTED;
                }
                lastA1 = gamepad1.a;
                break;
            case OUTTAKE_WRIST_RETRACTED:
                // retract
                if((gamepad1.a && !lastA1) || (retractRequested && timeSince(retractTime) > RETRACT_DELAY)) {
                    robotState = RobotState.RETRACT;
                    retractRequested = false;
                    outtake.outtaking = false;
                    outtake.lift.setMotionProfileTargetPos(0);
                }

                // extend wrist
                if(gamepad1.dpad_down && !lastDown1) {
                    retractRequested = false;
                    outtake.extendWrist();
                    robotState = RobotState.OUTTAKE_WRIST_UP;
                    outtakeTime = currentTime();
                }
                lastDown1 = gamepad1.dpad_down;

                // Change height
                if(gamepad1.x) {
                    retractRequested = false;
                    outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                }
                if(gamepad1.y) {
                    retractRequested = false;
                    outtake.setTargetHeight(Outtake.MED_HEIGHT);
                }
                if(gamepad1.b) {
                    retractRequested = false;
                    outtake.setTargetHeight(Outtake.HIGH_HEIGHT);
                }
                break;
        }

// DRIVING

        drivetrain.setDrivePower(robotState, gamepad1);

        double horz = gamepad1.left_stick_x;
        double vert = -gamepad1.left_stick_y;
        double rotate = -gamepad1.right_stick_x;

        if (gamepad1.left_stick_button) drivetrain.driveToHeading(horz, vert, boardHeading);
        else drivetrain.driveMaintainHeading(horz, vert, rotate);

        if(gamepad1.right_stick_button) {
            drivetrain.resetHeading(boardHeading);
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
        boardHeading = alliance == Alliance.RED ? Math.toRadians(180) : 0;
    }

    public void telemetry() {
        telemetry.addData("robot state", robotState);
    }
}

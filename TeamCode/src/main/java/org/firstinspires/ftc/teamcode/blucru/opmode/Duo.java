package org.firstinspires.ftc.teamcode.blucru.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.states.LiftState;
import org.firstinspires.ftc.teamcode.blucru.common.states.RobotState;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.blucru.common.util.BCLinearOpMode;


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
public class Duo extends BCLinearOpMode {
    // timer variables
    public static double OUTTAKE_TURN_TURRET_DELAY = 300;
    public static double RETRACT_WRIST_DELAY = 250;
    public static double FULL_RETRACT_DELAY = 350;
    public static double INTAKE_FULL_REVERSE_TIME = 1000;
    public static double START_INTAKE_READ_DELAY = 100;

    private RobotState robotState;

    double scoringHeading; // heading to score on board

    // timer variables
    double outtakeTime = 0;
    double retractTime = 0;
    double intakeFullTime = -10000; // init at -10000 ms to prevent immediate outtake
    double startIntakeTime = 0;

    // gamepad states
    boolean lastDown2 = false;
    boolean lastA2 = false;
    boolean lastRSUp2 = false;
    boolean lastRSDown2 = false;
    boolean lastUp1 = false;

    public void initialize() {
        robotState = RobotState.RETRACT;

        // add subsystems
        addDrivetrain(true);
        addOuttake();
        addIntake();
        addHanger();
        addPlane();

        scoringHeading = alliance == Alliance.RED ? Math.toRadians(180) : 0; // set the heading to score on the backboard
    }

    public void periodic() {
        // DRIVING
        drivetrain.setDrivePower(robotState, gamepad1);

        double horz = gamepad1.left_stick_x;
        double vert = -gamepad1.left_stick_y;
        double rotate = -gamepad1.right_stick_x;

        // resets heading offset (face away from board)
        if(gamepad1.right_stick_button) {
            drivetrain.resetHeading(scoringHeading);
            gamepad1.rumble(100);
        }

        if(gamepad1.b) drivetrain.driveToHeading(horz, vert, scoringHeading); // drive to outtake heading
        else if(gamepad1.x) drivetrain.driveToHeading(horz, vert, scoringHeading - Math.PI); // drive to opposite outtake heading
        else drivetrain.driveMaintainHeading(horz, vert, rotate); // drive normally

        switch(robotState) {
            case RETRACT:
                // LIFT
                if(gamepad2.x) {
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                }
                if(gamepad2.y) {
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.MED_HEIGHT);
                }
                if(gamepad2.b) {
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.HIGH_HEIGHT);
                }

                // INTAKE
                if(gamepad1.left_bumper && outtake.lift.intakeReady()) {
                    robotState = RobotState.INTAKING;
                    intake.startReadingColor();
                    startIntakeTime = currentTime();
                    outtake.unlock();
                    intake.intake();
                    intake.dropToGround();
                }

                if(gamepad1.a && outtake.lift.intakeReady()) {
                    robotState = RobotState.INTAKING;
                    intake.startReadingColor();
                    startIntakeTime = currentTime();
                    outtake.unlock();
                    intake.intake();
                    intake.dropToStack(2);
                }

                if(gamepad1.right_bumper) {
                    intake.setIntakePower(-1);
                } else {
                    intake.setIntakePower(0);
                }

                // REVERSE INTAKE
                if(timeSince(intakeFullTime) < INTAKE_FULL_REVERSE_TIME || gamepad2.right_bumper) {
                    intake.setIntakePower(-1);
                    intake.retractIntakeWrist();
                    outtake.lock();
                }
                break;
            case INTAKING:
                if(intake.isFull() && timeSince(startIntakeTime) > START_INTAKE_READ_DELAY) {
                    intakeFullTime = currentTime();
                    robotState = RobotState.RETRACT;
                    intake.stopReadingColor();
                    intake.setIntakePower(-1);
                    intake.retractIntakeWrist();
                    outtake.lock();
                } else if(gamepad1.left_bumper) {
                    intake.intake();
                    intake.dropToGround();
                } else if(gamepad1.a) {
                    intake.intake();
                    intake.dropToStack(2);
                } else {
                    outtake.lock();
                    intake.setIntakePower(0);
                    intake.retractIntakeWrist();
                    robotState = RobotState.RETRACT;
                    intake.stopReadingColor();
                }
                break;
            case LIFTING:
                if(outtake.liftWristClear()) {
                    outtake.wristRetracted = false;
                    robotState = RobotState.OUTTAKE_WRIST_UP;
                    outtakeTime = currentTime();
                }

                if(gamepad2.x) outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                if(gamepad2.y) outtake.setTargetHeight(Outtake.MED_HEIGHT);
                if(gamepad2.b) outtake.setTargetHeight(Outtake.HIGH_HEIGHT);

                if(gamepad2.a && !lastA2) {
                    robotState = RobotState.RETRACT;
                    outtake.retractLift();
                }
                lastA2 = gamepad2.a;

                // reverse intake
                if(gamepad1.right_bumper) {
                    intake.setIntakePower(-1);
                } else {
                    intake.setIntakePower(0);
                }
                break;
            case OUTTAKE_WRIST_UP:
                // TURRET CONTROL
                if(timeSince(outtakeTime) > OUTTAKE_TURN_TURRET_DELAY && !outtake.wristRetracted) {
                    if (gamepad2.left_trigger > 0.1) outtake.setTurretAngle(-gamepad2.left_trigger * 60 + 270);
                    else if (gamepad2.right_trigger > 0.1) outtake.setTurretAngle(gamepad2.right_trigger * 60 + 270);
                    else outtake.setTurretAngle(270);
                } else outtake.setTurretAngle(270);

                // retract wrist
                if(outtake.turret.isCentered() && gamepad2.dpad_down && !lastDown2) {
                    outtake.retractWrist();
                    robotState = RobotState.OUTTAKE_WRIST_RETRACTED;
                }
                lastDown2 = gamepad2.dpad_down;

                // Change height
                if(gamepad2.x) outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                if(gamepad2.y) outtake.setTargetHeight(Outtake.MED_HEIGHT);
                if(gamepad2.b) outtake.setTargetHeight(Outtake.HIGH_HEIGHT);

                // increment height by one pixel
                if(gamepad2.right_stick_y < -0.3 && !lastRSUp2 && !gamepad2.right_stick_button) outtake.incrementTargetHeight(1);
                lastRSUp2 = gamepad2.right_stick_y < -0.3;

                if(gamepad2.right_stick_y > 0.3 && !lastRSDown2 && !gamepad2.right_stick_button) outtake.incrementTargetHeight(-1);
                lastRSDown2 = gamepad2.right_stick_y > 0.3;

                // retract
                if(gamepad2.a && !lastA2) {
                    retractTime = currentTime();
                    outtake.retractWrist();
                    outtake.centerTurret();
                    outtake.incrementTargetHeight(1);
                    robotState = RobotState.RETRACTING;
                }
                lastA2 = gamepad2.a;

                // reverse intake
                if(gamepad1.right_bumper) {
                    intake.setIntakePower(-1);
                } else {
                    intake.setIntakePower(0);
                }
                break;
            case OUTTAKE_WRIST_RETRACTED:
                // retract
                if(gamepad2.a && !lastA2) {
                    robotState = RobotState.RETRACT;
                    outtake.retractLift();
                }
                lastA2 = gamepad2.a;

                // extend wrist
                if(gamepad2.dpad_down && !lastDown2) {
                    outtake.extendWrist();
                    robotState = RobotState.OUTTAKE_WRIST_UP;
                    outtakeTime = currentTime();
                }
                lastDown2 = gamepad2.dpad_down;

                // Change height
                if(gamepad2.x) outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                if(gamepad2.y) outtake.setTargetHeight(Outtake.MED_HEIGHT);
                if(gamepad2.b) outtake.setTargetHeight(Outtake.HIGH_HEIGHT);

                // increment height by one pixel
                if(gamepad2.right_stick_y < -0.3 && !lastRSUp2 && !gamepad2.right_stick_button) {
                    outtake.incrementTargetHeight(1);
                }
                lastRSUp2 = gamepad2.right_stick_y < -0.3;
                if(gamepad2.right_stick_y > 0.3 && !lastRSDown2 && !gamepad2.right_stick_button) {
                    outtake.incrementTargetHeight(-1);
                }
                lastRSDown2 = gamepad2.right_stick_y > 0.3;

                // reverse intake
                if(gamepad1.right_bumper) {
                    intake.setIntakePower(-1);
                } else {
                    intake.setIntakePower(0);
                }
                break;
            case RETRACTING:
                // retract wrist
                if(timeSince(retractTime) > RETRACT_WRIST_DELAY) {
                    outtake.retractWrist();
                    outtake.lock();
                }

                // fully retract
                if(timeSince(retractTime) > FULL_RETRACT_DELAY) {
                    outtake.retractLift();
                    robotState = RobotState.RETRACT;
                }

                // reverse intake
                if(gamepad1.right_bumper) {
                    intake.setIntakePower(-1);
                } else {
                    intake.setIntakePower(0);
                }
                break;
        }

        // MANUAL SLIDE
        if(Math.abs(gamepad2.right_stick_y) > 0.1 && gamepad2.right_stick_button) {
            outtake.lift.liftState = LiftState.MANUAL;
            outtake.setManualSlidePower(-gamepad2.right_stick_y + Lift.kF);
        }

        // RESET SLIDES
        if(gamepad2.share) {
            outtake.lift.resetEncoder();
        }

        // MANUAL HANG
        if(Math.abs(gamepad2.left_stick_y) > 0.2)
            hanger.setPower(-gamepad2.left_stick_y);
        else
            hanger.setPower(0);

        // PLANE
        if(gamepad1.dpad_up && !lastUp1)
            plane.togglePlane();
        lastUp1 = gamepad1.dpad_up;
    }

    public void telemetry() {
        telemetry.addData("robot state", robotState);
    }
}

package org.firstinspires.ftc.teamcode.blucru.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.states.RobotState;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.outtake.Outtake;


/*
Controls:

Gamepad 1:
    - left stick: drive translationally
    - right stick: rotate
    - right stick press: reset heading
    - b: drive to deposit heading
    - x: drive to opposite deposit heading
    - dpad up: toggle plane

Gamepad 2:

retracted state:
    - left bumper: intake ground level
        -GOES TO INTAKE STATE
    - a: intake stack level
        -GOES TO INTAKE STATE
    - right bumper: reverse intake

    - b: lift to high
        -GOES TO LIFTING STATE
    - x: lift to low
        -GOES TO LIFTING STATE
    - y: lift to mid
        -GOES TO LIFTING STATE

    - press right stick: manual slide control
    - share: reset slides

intaking state:
    - left bumper: intake ground level
    - a: intake stack level
    - right bumper: reverse intake
    - RELEASE GOES TO RETRACT

lifting state:
    - a: go to retract state
    - x: lift to low
    - y: lift to mid
    - b: lift to high
    - right bumper: reverse intake
    GOES TO OUTTAKING WHEN LIFT IS ABOVE WRIST CLEAR POSITION

outtaking state:
    - a: retract
        -GOES TO RETRACTING STATE
    - x: lift to low
    - y: lift to mid
    - b: lift to high
    - right bumper: reverse intake
    - right stick y: increment outtake height by one pixel
    - share: reset slides
    - dpad left: unlock front lock
    - dpad right: unlock all locks


 */
@TeleOp(name = "Main TeleOp", group = "1")
public class Duo extends BCLinearOpMode {
    // timer variables
    public static double OUTTAKE_TURN_TURRET_DELAY = 300,
            RETRACT_WRIST_DELAY = 250,
            FULL_RETRACT_DELAY = 350,
            INTAKE_FULL_REVERSE_TIME = 500;

    private RobotState robotState;

    double scoringHeading; // heading to score on board, teleop heading is driver centric, with 0 pointing to the right

    // timer variables
    double outtakeTime = 0,
            retractTime = 0,
            intakeFullTime = -10000, // init at -10000 ms to prevent immediate outtake
            startIntakeTime = 0;

    // gamepad states
    boolean lastDown2 = false,
            lastA2 = false,
            lastRSUp2 = false,
            lastRSDown2 = false,
            lastUp1 = false,
            lastManualSliding = false;

    public void initialize() {
        robotState = RobotState.RETRACT;

        // add subsystems
        addDrivetrain(true);
        addOuttake();
        addIntake();
        addHanger();
        addPlane();

        optimizeTelemetry();

        scoringHeading = alliance == Alliance.RED ? Math.toRadians(180) : 0; // set the heading to score on the backboard
        intake.startReadingColor();
    }

    public void periodic() {
        // DRIVING
        drivetrain.setTeleDrivePower(robotState, gamepad1);

        double horz = gamepad1.left_stick_x;
        double vert = -gamepad1.left_stick_y;
        double rotate = -gamepad1.right_stick_x;

        // resets heading offset (face away from board)
        if(gamepad1.right_stick_button) {
            drivetrain.resetHeading(scoringHeading);
            gamepad1.rumble(100);
        }

        if(gamepad1.b) drivetrain.driveToHeadingScaled(horz, vert, scoringHeading); // drive to outtake heading
        else if(gamepad1.x) drivetrain.driveToHeadingScaled(horz, vert, scoringHeading - Math.PI); // drive to opposite outtake heading
        else drivetrain.teleOpDrive(horz, vert, rotate); // drive normally

        switch(robotState) {
            case RETRACT:
                // LIFT
                if (gamepad2.x) {
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                    intake.stopReadingColor();
                }
                if (gamepad2.y) {
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.MED_HEIGHT);
                    intake.stopReadingColor();

                }
                if (gamepad2.b) {
                    robotState = RobotState.LIFTING;
                    outtake.setTargetHeight(Outtake.HIGH_HEIGHT);
                    intake.stopReadingColor();
                }

                // INTAKE
                if (gamepad2.left_bumper && outtake.lift.intakeReady() && !intake.isFull()) {
                    robotState = RobotState.INTAKING;
                    startIntakeTime = currentTime();
                    outtake.unlock();
                    intake.intake();
                    intake.dropToGround();
                } else if (gamepad2.a && outtake.lift.intakeReady() && !intake.isFull()) {
                    robotState = RobotState.INTAKING;
                    startIntakeTime = currentTime();
                    outtake.unlock();
                    intake.intake();
                    intake.dropToStack(3);

                    // REVERSE INTAKE
                } else if(timeSince(intakeFullTime) < INTAKE_FULL_REVERSE_TIME || gamepad2.right_bumper) {
                    intake.setIntakePower(-1);
                    intake.retractIntakeWrist();
                    outtake.lock();
                } else {
                    intake.setIntakePower(0);
                }

                // RESET SLIDES
                if(gamepad2.share) {
                    outtake.lift.resetEncoder();
                }
                break;
            case INTAKING:
                if(intake.isFull()) {
                    // rumble when full
                    gamepad2.rumble(150);
                    gamepad1.rumble(150);
                    intakeFullTime = currentTime();
                    robotState = RobotState.RETRACT;
                    intake.setIntakePower(-1);
                    intake.retractIntakeWrist();
                    outtake.lock();
                } else if(gamepad2.left_bumper) {
                    intake.intake();
                    intake.dropToGround();
                } else if(gamepad2.a) {
                    intake.intake();
                    intake.dropToStack(3);
                } else {
                    outtake.lock();
                    intake.setIntakePower(0);
                    intake.retractIntakeWrist();
                    robotState = RobotState.RETRACT;
                }
                break;
            case LIFTING:
                if(outtake.liftWristClear()) {
                    outtake.wristRetracted = false;
                    robotState = RobotState.OUTTAKING;
                    outtakeTime = currentTime();
                }

                if(gamepad2.x) outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                if(gamepad2.y) outtake.setTargetHeight(Outtake.MED_HEIGHT);
                if(gamepad2.b) outtake.setTargetHeight(Outtake.HIGH_HEIGHT);

                if(gamepad2.a && !lastA2) {
                    robotState = RobotState.RETRACT;
                    outtake.retractLift();
                    intake.startReadingColor();
                }
                lastA2 = gamepad2.a;

                // reverse intake
                if(gamepad2.right_bumper) {
                    intake.setIntakePower(-1);
                } else {
                    intake.setIntakePower(0);
                }
                break;
            case OUTTAKING:
                // TURRET CONTROL
                if(timeSince(outtakeTime) > OUTTAKE_TURN_TURRET_DELAY && !outtake.wristRetracted && Math.abs(gamepad2.right_stick_x) > 0.05) {
                    outtake.teleOpTurnTurret(gamepad2.right_stick_x);
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
                if(gamepad2.right_stick_y < -0.5 && !lastRSUp2 && !gamepad2.right_stick_button) outtake.incrementTargetHeight(1);
                lastRSUp2 = gamepad2.right_stick_y < -0.5;

                if(gamepad2.right_stick_y > 0.5 && !lastRSDown2 && !gamepad2.right_stick_button) outtake.incrementTargetHeight(-1);
                lastRSDown2 = gamepad2.right_stick_y > 0.5;

                // retract
                if(gamepad2.a && !lastA2) {
                    retractTime = currentTime();
                    outtake.retractWrist();
                    outtake.centerTurret();
                    outtake.incrementTargetHeight(1);
                    intake.startReadingColor();
                    robotState = RobotState.RETRACTING;
                }
                lastA2 = gamepad2.a;

                // reverse intake
                if(gamepad2.right_bumper) intake.setIntakePower(-1);
                else intake.setIntakePower(0);

                if(gamepad2.dpad_left) outtake.lock.unlockFrontLockBack();
                else if (gamepad2.dpad_right) outtake.lock.unlockAll();



                break;
            case OUTTAKE_WRIST_RETRACTED:
                // retract
                if(gamepad2.a && !lastA2) {
                    robotState = RobotState.RETRACT;
                    outtake.retractLift();
                    outtake.resetLock();
                    intake.startReadingColor();
                }
                lastA2 = gamepad2.a;

                // extend wrist
                if(gamepad2.dpad_down && !lastDown2) {
                    outtake.extendWrist();
                    robotState = RobotState.OUTTAKING;
                    outtakeTime = currentTime();
                }
                lastDown2 = gamepad2.dpad_down;

                // Change height
                if(gamepad2.x) outtake.setTargetHeight(Outtake.LOW_HEIGHT);
                if(gamepad2.y) outtake.setTargetHeight(Outtake.MED_HEIGHT);
                if(gamepad2.b) outtake.setTargetHeight(Outtake.HIGH_HEIGHT);

                // increment height by one pixel
                if(gamepad2.right_stick_y < -0.5 && !lastRSUp2 && !gamepad2.right_stick_button) {
                    outtake.incrementTargetHeight(1);
                }
                lastRSUp2 = gamepad2.right_stick_y < -0.5;
                if(gamepad2.right_stick_y > 0.5 && !lastRSDown2 && !gamepad2.right_stick_button) {
                    outtake.incrementTargetHeight(-1);
                }
                lastRSDown2 = gamepad2.right_stick_y > 0.5;

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
                    intake.startReadingColor();
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
            outtake.setManualSlidePower(-gamepad2.right_stick_y);
        }
        if(lastManualSliding && Math.abs(gamepad2.right_stick_y) < 0.1 && !gamepad2.right_stick_button) {
            outtake.stopManualSlide();
        }
        lastManualSliding = Math.abs(gamepad2.right_stick_y) > 0.1 && gamepad2.right_stick_button;

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

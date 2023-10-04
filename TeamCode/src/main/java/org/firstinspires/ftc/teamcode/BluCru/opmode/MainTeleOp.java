package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.BluCru.Hardware6417;


/*
Controls:

release everything: retracted state
a : release cone/intake
b : low height
x : medium height
y : high height
right trigger (hold) : turret to the right, then wrist down
left trigger (hold) : turret to the left, then wrist down
left joystick : strafe (field centric)
right joystick : turn
options : cone righting + snail drive

 */
@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends LinearOpMode {
    Hardware6417 robot;
    enum SLIDESTATE {
        zero, low, medium, high, coneRight, manual
    }
    enum TURRETSTATE {
        center, right, left,
    }
    enum WRISTSTATE {
        down, up, coneRight
    }
    enum TWISTERSTATE {
        right, left, center
    }
    enum ROBOTSTATE{
        intake,
        maneuvering,
        outtakeQueueLow,
        outtakeQueueMedium,
        outtakeQueueHigh,
        outtakeReadyLow,
        outtakeReadyMedium,
        outtakeReadyHigh,
        coneRight
    }

    SLIDESTATE slideState;
    TURRETSTATE turretState;
    WRISTSTATE wristState;
    TWISTERSTATE twisterState;
    ROBOTSTATE robotState, lastRobotState;

    Gamepad currentGamepad1, currentGamepad2, lastGamepad1, lastGamepad2;

    double slideZeroTime;
    double turnTurretTime;
    double grabTime;
    double timeSinceSlideZero, timeSinceTurretTurn, timeSinceGrab;
    double vertControl, horzControl, rotateControl;
    double driveSpeed;
    double heading, headingOffset;
    boolean grabbing;

    ElapsedTime totalTimer;

    int numOfGamepads;
    @Override
    public void runOpMode() throws InterruptedException {
        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        lastGamepad1 = new Gamepad();
        lastGamepad2 = new Gamepad();
        robot = new Hardware6417(hardwareMap);

        grabbing = false;
        int dunk = 0;

        slideZeroTime = 0;
        turnTurretTime = Double.POSITIVE_INFINITY;
        grabTime = Double.POSITIVE_INFINITY;
        driveSpeed = 0;
//        heading = robot.getRawExternalHeading();
        headingOffset = 0;

        initStates();
        initRobot();

        telemetry.addData("Status", "Initialized");

        waitForStart();

        // turret sometimes doesnt lock
        turretState = TURRETSTATE.center;

//        headingOffset = robot.getRawExternalHeading() + Math.toRadians(180);
        totalTimer = new ElapsedTime();

        while(opModeIsActive()) {
            setNumOfGamepads();
            lastGamepad1.copy(currentGamepad1);
            lastGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            switch (robotState) {
                case intake:
                    setIntakeStates();
                    // hold a
                    if(currentGamepad1.a) {
                        grabbing = false;
                    } else {
                        grabbing = true;
                        processIntakeSequence();
                    }
                    break;
                case maneuvering:
                    setManeuveringStates();

                    // go to intake
                    if(currentGamepad1.a && robot.sliderIntakeReady()) {
                        setRobotState(ROBOTSTATE.intake);
                    }

                    // outtake slider presets
                    if(currentGamepad1.b && !lastGamepad1.b) {
                        setRobotState(ROBOTSTATE.outtakeQueueLow);
                    }
                    if(currentGamepad1.x && !lastGamepad1.x) {
                        setRobotState(ROBOTSTATE.outtakeQueueMedium);
                    }
                    if(currentGamepad1.y && !lastGamepad1.y) {
                        setRobotState(ROBOTSTATE.outtakeQueueHigh);
                    }

                    // cone righting
                    if(currentGamepad1.options && !lastGamepad1.options) {
                        setRobotState(ROBOTSTATE.coneRight);
                        wristState = WRISTSTATE.down;
                        grabbing = false;
                    }
                    break;
                case outtakeQueueLow:
                    setOuttakeQueueStates();
                    slideState = SLIDESTATE.low;

                    // TURNING TURRET
                    processOuttakeQueueTurret();

                    if(outtakeReady()) {
                        wristState = WRISTSTATE.down;
                        setRobotState(ROBOTSTATE.outtakeReadyLow);
                    }

                    // OTHER SLIDE PRESETS
                    if(currentGamepad1.x && !lastGamepad1.x) {
                        setRobotState(ROBOTSTATE.outtakeQueueMedium);
                    }
                    if(currentGamepad1.y && !lastGamepad1.y) {
                        setRobotState(ROBOTSTATE.outtakeQueueHigh);
                    }
                    if(currentGamepad1.a && !lastGamepad1.a) {
                        setRobotState(ROBOTSTATE.maneuvering);
                        slideZeroTime = totalTimer.seconds();
                    }

                    // toggle grabber
                    if(currentGamepad1.left_bumper && !lastGamepad1.left_bumper) {
                        grabbing = !grabbing;
                    }

                    // toggle wrist
                    if(currentGamepad1.dpad_down && !lastGamepad1.dpad_down) {
                        toggleWrist();
                    }
                    break;
                case outtakeQueueMedium:
                    setOuttakeQueueStates();
                    slideState = SLIDESTATE.medium;

                    processOuttakeQueueTurret();

                    if(outtakeReady()) {
                        wristState = WRISTSTATE.down;
                        setRobotState(ROBOTSTATE.outtakeReadyMedium);
                    }

                    // OTHER SLIDE PRESETS
                    if(currentGamepad1.b && !lastGamepad1.b) {
                        setRobotState(ROBOTSTATE.outtakeQueueLow);
                    }
                    if(currentGamepad1.y && !lastGamepad1.y) {
                        setRobotState(ROBOTSTATE.outtakeQueueHigh);
                    }
                    if(currentGamepad1.a && !lastGamepad1.a) {
                        setRobotState(ROBOTSTATE.maneuvering);
                        slideZeroTime = totalTimer.seconds();
                    }

                    // toggle grabber
                    if(currentGamepad1.left_bumper && !lastGamepad1.left_bumper) {
                        grabbing = !grabbing;
                    }

                    // toggle wrist
                    if(currentGamepad1.dpad_down && !lastGamepad1.dpad_down) {
                        toggleWrist();
                    }
                    break;
                case outtakeQueueHigh:
                    setOuttakeQueueStates();
                    slideState = SLIDESTATE.high;

                    // TURNING TURRET
                    processOuttakeQueueTurret();

                    if(outtakeReady()) {
                        wristState = WRISTSTATE.down;
                        setRobotState(ROBOTSTATE.outtakeReadyHigh);
                    }

                    // OTHER SLIDE PRESETS
                    if(currentGamepad1.b && !lastGamepad1.b) {
                        setRobotState(ROBOTSTATE.outtakeQueueLow);
                    }
                    if(currentGamepad1.x && !lastGamepad1.x) {
                        setRobotState(ROBOTSTATE.outtakeQueueMedium);
                    }
                    if(currentGamepad1.a && !lastGamepad1.a) {
                        setRobotState(ROBOTSTATE.maneuvering);
                        slideZeroTime = totalTimer.seconds();
                    }

                    // toggle grabber
                    if(currentGamepad1.left_bumper && !lastGamepad1.left_bumper) {
                        grabbing = !grabbing;
                    }

                    // toggle wrist
                    if(currentGamepad1.dpad_down && !lastGamepad1.dpad_down) {
                        toggleWrist();
                    }
                    break;
                case outtakeReadyLow:
                    twisterState = TWISTERSTATE.center;
                    slideState = SLIDESTATE.low;
                    driveSpeed = Constants.driveSpeedOuttakeReady;

                    processOuttakeReadyStates(ROBOTSTATE.outtakeQueueLow);

                    if(currentGamepad1.a && !lastGamepad1.a) {
                        setRobotState(ROBOTSTATE.maneuvering);
                        slideZeroTime = totalTimer.seconds();
                    }

                    if(currentGamepad1.x && !lastGamepad1.x) {
                        setRobotState(ROBOTSTATE.outtakeQueueMedium);
                    }
                    if(currentGamepad1.y && !lastGamepad1.y) {
                        setRobotState(ROBOTSTATE.outtakeQueueHigh);
                    }

                    // dunk slides
                    if(currentGamepad1.right_bumper) {
                        dunk = Constants.sliderDunkDelta;
                    } else {
                        dunk = 0;
                    }

                    // toggle grabber
                    if(currentGamepad1.left_bumper && !lastGamepad1.left_bumper) {
                        grabbing = !grabbing;
                    }

                    // toggle wrist
                    if(currentGamepad1.dpad_down && !lastGamepad1.dpad_down) {
                        toggleWrist();
                    }
                    break;
                case outtakeReadyMedium:
                    twisterState = TWISTERSTATE.center;
                    slideState = SLIDESTATE.medium;
                    driveSpeed = Constants.driveSpeedOuttakeReady;

                    processOuttakeReadyStates(ROBOTSTATE.outtakeQueueMedium);

                    if(currentGamepad1.a && !lastGamepad1.a) {
                        setRobotState(ROBOTSTATE.maneuvering);
                        slideZeroTime = totalTimer.seconds();
                    }
                    if(currentGamepad1.b && !lastGamepad1.b) {
                        setRobotState(ROBOTSTATE.outtakeQueueLow);
                    }
                    if(currentGamepad1.y && !lastGamepad1.y) {
                        setRobotState(ROBOTSTATE.outtakeQueueHigh);
                    }

                    // dunk slides
                    if(currentGamepad1.right_bumper) {
                        dunk = Constants.sliderDunkDelta;
                    } else {
                        dunk = 0;
                    }

                    // toggle grabber
                    if(currentGamepad1.left_bumper && !lastGamepad1.left_bumper) {
                        grabbing = !grabbing;
                    }

                    // toggle wrist
                    if(currentGamepad1.dpad_down && !lastGamepad1.dpad_down) {
                        toggleWrist();
                    }
                    break;
                case outtakeReadyHigh:
                    twisterState = TWISTERSTATE.center;
                    slideState = SLIDESTATE.high;
                    driveSpeed = Constants.driveSpeedOuttakeReady;

                    processOuttakeReadyStates(ROBOTSTATE.outtakeQueueHigh);

                    if(currentGamepad1.a && !lastGamepad1.a) {
                        setRobotState(ROBOTSTATE.maneuvering);
                        slideZeroTime = totalTimer.seconds();
                    }
                    if(currentGamepad1.b && !lastGamepad1.b) {
                        setRobotState(ROBOTSTATE.outtakeQueueLow);
                    }
                    if(currentGamepad1.x && !lastGamepad1.x) {
                        setRobotState(ROBOTSTATE.outtakeQueueMedium);
                    }

                    // dunk slides
                    if(currentGamepad1.right_bumper) {
                        dunk = Constants.sliderDunkDelta;
                    } else {
                        dunk = 0;
                    }

                    // toggle grabber
                    if(currentGamepad1.left_bumper && !lastGamepad1.left_bumper) {
                        grabbing = !grabbing;
                    }

                    // toggle wrist
                    if(currentGamepad1.dpad_down && !lastGamepad1.dpad_down) {
                        toggleWrist();
                    }
                    break;
                case coneRight:
                    slideState = SLIDESTATE.coneRight;
                    turretState = TURRETSTATE.center;
                    twisterState = TWISTERSTATE.center;
                    driveSpeed = Constants.driveSpeedConeRight;

                    if(currentGamepad1.a && !lastGamepad1.a) {
                        setRobotState(ROBOTSTATE.maneuvering);
                    }

                    // toggle between wrist down and cone right
                    if(gamepad1.right_bumper && !lastGamepad1.right_bumper) {
                        if(wristState == WRISTSTATE.down) {
                            wristState = WRISTSTATE.coneRight;
                        } else {
                            wristState = WRISTSTATE.down;
                        }
                    }

                    if(gamepad1.left_bumper && !lastGamepad1.left_bumper) {
                        grabbing = !grabbing;
                    }
                    break;
            }

            // reset heading
            if(currentGamepad1.right_stick_button && !lastGamepad1.right_stick_button) {
//                headingOffset = robot.getExternalHeading() + Math.toRadians(180);
            }

            vertControl = Math.pow(-gamepad1.left_stick_y, 3);
            horzControl = Math.pow(gamepad1.left_stick_x, 3);
            rotateControl = Math.pow(-gamepad1.right_stick_x, 3);
//            heading = robot.getExternalHeading();

            if(Math.max(Math.abs(vertControl), Math.max(Math.abs(horzControl), Math.abs(rotateControl))) > 0.1) {
//                robot.holonomicDrive(vertControl, horzControl, rotateControl, driveSpeed, heading - headingOffset);
            } else {
//                robot.holonomicDrive(0,0,0,0,0);
            }

            // slider control
            switch (slideState) {
                case zero:
                    timeSinceSlideZero = totalTimer.seconds() - slideZeroTime;
                    if(timeSinceSlideZero > Constants.slideStallDelay){
                        robot.resetSliders();
                    } else {
                        robot.autoSlide(0, Constants.slideBasePower);
                    }
                    break;
                case low:
                    robot.autoSlide(Constants.sliderLowPos - dunk, Constants.slideLowPower);
                    break;
                case medium:
                    robot.autoSlide(Constants.sliderMedPos - dunk, Constants.slideMedPower);
                    break;
                case high:
                    robot.autoSlide(Constants.sliderHighPos - dunk, Constants.slideHighPower);
                    break;
                case coneRight:
                    robot.autoSlide(Constants.sliderConeRightPos, Constants.slideBasePower);
                    break;
            }

            // wrist control
            switch (wristState) {
                case up:
                    robot.autoWrist(Constants.wristUpPos);
                    break;
                case down:
                    robot.autoWrist(Constants.wristDownPos);
                    break;
                case coneRight:
                    robot.autoWrist(Constants.wristConeRightPos);
                    break;
            }

            // twister control
            switch(twisterState) {
                case center:
                    robot.autoTwister(Constants.twisterCenterPos);
                    break;
                case left:
                    robot.autoTwister(Constants.twisterLeftPos);
                    break;
                case right:
                    robot.autoTwister(Constants.twisterRightPos);
                    break;
            }

            // turret control
            switch(turretState) {
                case center:
                    robot.autoTurret(Constants.turretCenterPos);
                    break;
                case left:
                    if(robot.turretClear()) {
                        robot.autoTurret(Constants.turretLeftPos);
                    } else {
                        robot.autoTurret(Constants.turretCenterPos);
                    }
                    break;
                case right:
                    if(robot.turretClear()) {
                        robot.autoTurret(Constants.turretRightPos);
                    } else {
                        robot.autoTurret(Constants.turretCenterPos);
                    }
                    break;
            }

            // grabber control
            if(grabbing) {
                robot.closeGrabber();
            } else {
                robot.openGrabber();
            }

            telemetry.addData("robotState: ", robotState);
            telemetry.addData("slideState: ", slideState);
            telemetry.addData("turretState: ", turretState);
            telemetry.addData("wristState: ", wristState);
            telemetry.addData("twisterState: ", twisterState);
            telemetry.addData("driveSpeed: ", driveSpeed);
            telemetry.addData("dunk: ", dunk);
            telemetry.addData("heading: ", heading);
            robot.telemetry(telemetry);
            telemetry.update();
        }
    }


    public void setIntakeStates() {
        slideState = SLIDESTATE.zero;
        wristState = WRISTSTATE.down;
        twisterState = TWISTERSTATE.center;
        turretState = TURRETSTATE.center;
        driveSpeed = Constants.driveSpeedIntake;
    }
    public void processIntakeSequence(){
        if(lastGamepad1.a) {
            grabTime = totalTimer.seconds();
        }
        timeSinceGrab = totalTimer.seconds() - grabTime;
        if(timeSinceGrab > Constants.wristGrabDelay) {
            setRobotState(ROBOTSTATE.maneuvering);
        }
    }

    public void setManeuveringStates() {
        slideState = SLIDESTATE.zero;
        wristState = WRISTSTATE.up;
        twisterState = TWISTERSTATE.center;
        turretState = TURRETSTATE.center;
        driveSpeed = Constants.driveSpeedManeuvering;
    }

    public void setOuttakeQueueStates() {
        twisterState = TWISTERSTATE.center;
        driveSpeed = Constants.driveSpeedOuttakeQueue;
    }

    public void processOuttakeQueueTurret() {
        if(currentGamepad1.right_trigger > 0.1 && gamepad1.left_trigger > 0.1){
            turretState = TURRETSTATE.center;        // if both triggers pressed, center turret
        } else if(currentGamepad1.right_trigger > 0.1) {
            turretState = TURRETSTATE.right;
            if(!(lastGamepad1.right_trigger > 0.1)){
                turnTurretTime = totalTimer.seconds();
            }
            timeSinceTurretTurn = totalTimer.seconds() - turnTurretTime;
        } else if(currentGamepad1.left_trigger > 0.1) {
            turretState = TURRETSTATE.left;
            if(!(lastGamepad1.left_trigger > 0.1)) {
                turnTurretTime = totalTimer.seconds();
            }
            timeSinceTurretTurn = totalTimer.seconds() - turnTurretTime;
        } else {
            turretState = TURRETSTATE.center;
        }
    }

    public boolean outtakeReady() {
        return turretState != TURRETSTATE.center &&
                timeSinceTurretTurn > Constants.wristTurretTurnDelay &&
                robot.slideOuttakeReady();
    }

    public void processOuttakeReadyStates(ROBOTSTATE outtakeQueue) {
        if(currentGamepad1.left_trigger > 0.1 && currentGamepad1.right_trigger > 0.1) {
            wristState = WRISTSTATE.up;
            setRobotState(outtakeQueue);
        } else if(currentGamepad1.left_trigger > 0.1) {
            turretState = TURRETSTATE.left;
        } else if(currentGamepad1.right_trigger > 0.1) {
            turretState = TURRETSTATE.right;
        } else {
            wristState = WRISTSTATE.up;
            setRobotState(outtakeQueue);
        }
    }

    public void toggleWrist() {
        if(wristState == WRISTSTATE.up) {
            wristState = WRISTSTATE.down;
        } else {
            wristState = WRISTSTATE.up;
        }
    }
    public void setRobotState(ROBOTSTATE targetRobotState) {
        lastRobotState = robotState;
        robotState = targetRobotState;
    }

    public void setNumOfGamepads() {
        if(gamepad2.getGamepadId() == -1) {
            numOfGamepads = 1;
        } else {
            numOfGamepads = 2;
        }
    }

    public void initStates() {
        slideState = SLIDESTATE.zero;
        turretState = TURRETSTATE.center;
        wristState = WRISTSTATE.up;
        twisterState = TWISTERSTATE.center;
        robotState = ROBOTSTATE.maneuvering;
    }

    public void initRobot() {
        robot.retractOdo();
        robot.resetSliders();
        robot.autoWrist(Constants.wristUpPos);
        robot.openGrabber();
        robot.autoTurret(Constants.turretCenterPos);
        robot.autoTwister(Constants.twisterCenterPos);
    }
}

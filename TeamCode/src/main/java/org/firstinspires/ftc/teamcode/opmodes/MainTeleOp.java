package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware6417;


/*
Controls:

release everything: retracted state
a : release cone/intake
b : low height
x : medium height
y : high height
right trigger (hold) : turret to the right
left trigger (hold) : turret to the left
left joystick : strafe (field centric)
right joystick : turn
left joystick (press) : cone righting + snail drive

 */
@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends LinearOpMode {
    Hardware6417 robot;
    enum SLIDESTATE {
        zero,
        low,
        medium,
        high,
        coneright,
        manual
    }
    enum TURRETSTATE {
        center,
        right,
        left,
    }
    enum WRISTSTATE {
        down,
        up,
        coneRight
    }
    enum TWISTERSTATE {
        counterClockwise,
        clockwise,
        center
    }
    enum ROBOTSTATE{
        intake,
        maneuvering,
        outtake,
        coneRight
    }

    SLIDESTATE slideState;
    TURRETSTATE turretState;
    WRISTSTATE wristState;
    TWISTERSTATE twisterState;
    ROBOTSTATE robotState;
    ROBOTSTATE lastRobotState;

    int numOfGamepads;
    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad lastGamepad1 = new Gamepad();
        Gamepad lastGamepad2 = new Gamepad();
        initStates();
        robot = new Hardware6417(hardwareMap);

        boolean grabbing = false;
        int dunk = 0;

        double slideZeroTime = -100;
        double timeSinceSlideZero;
        double turnTurretTime = -100;
        double timeSinceTurretTurn;
        double grabTime = -100;
        double switchStateTime = -100;
        double timeSinceGrab;

        waitForStart();

        // robot.retractOdo();
        ElapsedTime totalTimer = new ElapsedTime();

        while(opModeIsActive()) {
            setNumOfGamepads();
            timeSinceSlideZero = totalTimer.seconds() - slideZeroTime;
            timeSinceTurretTurn = totalTimer.seconds() - turnTurretTime;


            switch (robotState) {
                case intake:
                    slideState = SLIDESTATE.zero;
                    wristState = WRISTSTATE.down;
                    twisterState = TWISTERSTATE.center;
                    turretState = TURRETSTATE.center;

                    if(gamepad1.a) {
                        grabbing = false;
                    } else {
                        grabbing = true;
                        if(lastGamepad1.a) {
                            grabTime = totalTimer.seconds();
                        }
                        timeSinceGrab = totalTimer.seconds() - grabTime;
                        if(timeSinceGrab > Constants.wristGrabDelay) {
                            setRobotState(ROBOTSTATE.maneuvering);
                        }
                    }
                    break;
                case maneuvering:
                    slideState = SLIDESTATE.zero;
                    wristState = WRISTSTATE.up;
                    twisterState = TWISTERSTATE.center;
                    grabbing = true;

                    if(gamepad1.a && !lastGamepad1.a && robot.sliderIntakeReady()) {
                        setRobotState(ROBOTSTATE.intake);
                    }

                    if(gamepad1.b && !lastGamepad1.b) {
                        slideState = SLIDESTATE.low;
                        setRobotState(ROBOTSTATE.outtake);
                    }
                    if(gamepad1.x && !lastGamepad1.x) {
                        slideState = SLIDESTATE.medium;
                        setRobotState(ROBOTSTATE.outtake);
                    }
                    if(gamepad1.y && !lastGamepad1.y) {
                        slideState = SLIDESTATE.high;
                        setRobotState(ROBOTSTATE.outtake);
                    }
                    break;
                case outtake:
                    if(gamepad1.right_trigger > 0.1 && gamepad1.left_trigger > 0.1){
                        turretState = TURRETSTATE.center;
                    } else if(gamepad1.right_trigger > 0.1) {
                        turretState = TURRETSTATE.right;
                        if(!(lastGamepad1.right_trigger > 0.1)){
                            turnTurretTime = totalTimer.seconds();
                        }
                    }
                    if(gamepad1.a && !lastGamepad1.a) {
                        setRobotState(ROBOTSTATE.maneuvering);
                    }
                    break;
                case coneRight:
                    break;
            }

            // slider control
            if(gamepad1.a) {
                slideState = SLIDESTATE.zero;
                turretState = TURRETSTATE.center;

                if(true) {
                    wristState = WRISTSTATE.down;
                    grabbing = false;
                }
                if(!lastGamepad1.a) {
                    slideZeroTime = totalTimer.seconds();
                }
            }
            if(lastGamepad1.a && !gamepad1.a) {
                grabbing = true;
                grabTime = totalTimer.seconds();
            }
            if(gamepad1.b && !lastGamepad1.b) {
                slideState = SLIDESTATE.low;
            }
            if(gamepad1.x && !lastGamepad1.x) {
                slideState = SLIDESTATE.medium;
            }
            if(gamepad1.y && !lastGamepad1.y) {
                slideState = SLIDESTATE.high;
            }
            // dunk
            if(gamepad1.right_bumper) {
                dunk = Constants.sliderDunkDelta;
            } else {
                dunk = 0;
            }

            // cone righting
            if(gamepad1.left_stick_button && !lastGamepad1.left_stick_button) {
                slideState = SLIDESTATE.coneright;
                wristState = WRISTSTATE.coneRight;
            }

            // turret control
            if(robot.turretClear() && gamepad1.left_trigger > 0.1 && !(lastGamepad1.left_trigger > 0.1)) {
                turretState = TURRETSTATE.left;
                turnTurretTime = totalTimer.seconds();
            } else if (robot.turretClear() && gamepad1.right_trigger > 0.1 && !(lastGamepad1.right_trigger > 0.1)) {
                turretState = TURRETSTATE.right;
                turnTurretTime = totalTimer.seconds();
            } else {
                turretState = TURRETSTATE.center;
            }

            // grabbing
            if(gamepad1.left_bumper && !lastGamepad1.left_bumper) {
                grabbing = !grabbing;
            }

            switch (slideState) {
                case zero:
                    if(timeSinceSlideZero > 4 && timeSinceSlideZero < 6){
                        robot.resetSliders();
                    } else {
                        robot.autoSlide(0, Constants.slideBasePower);
                    }
                    break;
                case low:
                    robot.autoSlide(Constants.sliderLowPos - dunk, Constants.slideLowPower);
                    if(timeSinceTurretTurn > 0.7 && timeSinceTurretTurn < 1) {
                        wristState = WRISTSTATE.down;
                    }
                    break;
                case medium:
                    robot.autoSlide(Constants.sliderMedPos, Constants.slideMedPower);
                    if(timeSinceTurretTurn > 0.7 && timeSinceTurretTurn < 1) {
                        wristState = WRISTSTATE.down;
                    }
                    break;
                case high:
                    robot.autoSlide(Constants.sliderHighPos, Constants.slideHighPower);
                    if(timeSinceTurretTurn > 0.7 && timeSinceTurretTurn < 1) {
                        wristState = WRISTSTATE.down;
                    }
                    break;
            }


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

            /*switch(wristState) {
                case straight:
                    robot.autoWrist(Constants.wristStraightPos);
                    break;
                case clockwise:
                    robot.autoWrist(Constants.wristClockwisePos);
                    break;
                case counterClockwise:
                    robot.autoWrist(Constants.wristCounterClockwisePos);
                    break;
            }*/

            switch(turretState) {
                case center:
                    robot.autoTurret(Constants.turretCenterPos);
                    break;
                case left:
                    robot.autoTurret(Constants.turretLeftPos);
                    break;
                case right:
                    robot.autoTurret(Constants.turretRightPos);
                    break;
            }

            if(grabbing) {
                robot.closeGrabber();
            } else {
                robot.openGrabber();
            }

            setLastGamepads(lastGamepad1, lastGamepad2);
        }
    }

    public void setLastGamepads(Gamepad lastG1, Gamepad lastG2) {
        lastG1.copy(gamepad1);
        lastG2.copy(gamepad2);
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
}

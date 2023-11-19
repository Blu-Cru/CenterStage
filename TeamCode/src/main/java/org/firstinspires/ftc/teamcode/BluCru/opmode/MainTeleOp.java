package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BluCru.Constants;
import org.firstinspires.ftc.teamcode.BluCru.Hardware6417;


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
@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends LinearOpMode {
    Hardware6417 robot;
    enum ROBOTSTATE{
        moving, intake, eject, preOuttake, outtake, preHang, hanging
    }

    enum SLIDESTATE {
        zero, intake, low, med, high, manual
    }

    enum WRISTSTATE{
        moving, intake, preOuttake, outtake
    }

    enum WHEELSTATE{
        intake, outtake, stop
    }

    ROBOTSTATE robotState, lastRobotState;
    SLIDESTATE slideState;
    WRISTSTATE wristState;
    WHEELSTATE wheelState;

    Gamepad currentGamepad1, currentGamepad2, lastGamepad1, lastGamepad2;

    double slideZeroTime;
    double vert, horz, rotate;
    double driveSpeed;
    double heading, headingOffset;
    double sens = 0.1;
    boolean duoDrive;

    ElapsedTime totalTimer;
    double lastTime, deltaTime;

    @Override
    public void runOpMode() throws InterruptedException {
        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        lastGamepad1 = new Gamepad();
        lastGamepad2 = new Gamepad();
        robot = new Hardware6417(hardwareMap);
        robot.initSlides();
        robot.initWrist();
        robot.initWheels();
        robot.initDrive(new Pose2d(0, 0, 0));

        slideZeroTime = 0;
        driveSpeed = 0;
//        heading = robot.getRawExternalHeading();
        headingOffset = 0;

        initStates();
        initRobot();

        telemetry.addData("Status", "Initialized");

        waitForStart();

//        headingOffset = robot.getRawExternalHeading() + Math.toRadians(180);
        totalTimer = new ElapsedTime();

        while(opModeIsActive()) {
            setNumOfGamepads();
            // gets time at the start of loop
            lastTime = totalTimer.milliseconds();

            // sets gamepad for the loop
            lastGamepad1.copy(currentGamepad1);
            lastGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

// driving
            horz = Math.pow(gamepad1.left_stick_x, 3);
            vert = -Math.pow(gamepad1.left_stick_y, 3);
            rotate = -Math.pow(gamepad1.right_stick_x, 3);

            robot.holonomicDrive(horz, vert, rotate, driveSpeed, heading);

// wheel control
            if(gamepad1.left_trigger > Constants.triggerSens) {
                robot.setWheelPowers(-Constants.wheelOuttakePower * gamepad1.left_trigger);
            } else if(gamepad1.right_trigger > Constants.triggerSens) {
                robot.setWheelPowers(Constants.wheelIntakePower * gamepad1.right_trigger);
            } else {
                robot.setWheelPowers(0);
            }
            
// robot superstate control
            switch (robotState) {
                case moving:
                    setSubstates(SLIDESTATE.zero, WRISTSTATE.moving, Constants.driveSpeedMoving);
                    // press right trigger to intake
                    if(currentGamepad1.right_trigger > Constants.triggerSens) {
                        setRobotState(ROBOTSTATE.eject);
                    // press left trigger to eject
                    } else if (currentGamepad1.left_trigger > Constants.triggerSens && !(lastGamepad1.left_trigger > Constants.triggerSens)) {
                        setRobotState(ROBOTSTATE.intake);
                    }

                    break;
                case intake:
                    setSubstates(SLIDESTATE.intake, WRISTSTATE.intake, Constants.driveSpeedIntake);
                    if(!(currentGamepad1.left_trigger > Constants.triggerSens)) {
                        setRobotState(ROBOTSTATE.moving);
                    }
                    break;
                case eject:
                    setSubstates(SLIDESTATE.intake, WRISTSTATE.intake, Constants.driveSpeedEject);
                    if(!currentGamepad1.right_bumper && lastGamepad1.right_bumper) {
                        setRobotState(ROBOTSTATE.moving);
                    }
                    break;
                case outtake:
                    setSubstates(SLIDESTATE.med, WRISTSTATE.outtake, Constants.driveSpeedPreOuttake);
                    if(!currentGamepad1.right_bumper && lastGamepad1.right_bumper) {
                        setRobotState(ROBOTSTATE.outtake);
                    }
                    break;
            }

            // slide control
            switch(slideState) {
                case zero:
                    if(currentGamepad1.a && !lastGamepad1.a) {
                        slideZeroTime = totalTimer.milliseconds();
                    }
                    if(totalTimer.milliseconds() - slideZeroTime > Constants.slideStallDelay) {
                        robot.resetSliders();
                    } else {
                        robot.autoSlider(Constants.sliderBasePos);
                    }
                    break;
                case low:
                    robot.autoSlider(Constants.sliderLowPos);
                    break;
                case med:
                    robot.autoSlider(Constants.sliderMedPos);
                    break;
                case high:
                    robot.autoSlider(Constants.sliderHighPos);
                    break;
                case manual:
                    break;
            }

            // wrist control
            switch(wristState) {
                case moving:
                    robot.autoWrist(Constants.wristMovingPos);
                    break;
                case intake:
                    robot.stopWrist();
                    break;
                case preOuttake:
                    robot.autoWrist(Constants.wristPreOuttakePos);
                    break;
                case outtake:
                    robot.autoWrist(Constants.wristOuttakePos);
                    break;
            }



            // loop time: current time - time at start of loop
            deltaTime = totalTimer.milliseconds() - lastTime;

            robot.telemetry(telemetry);
            telemetry.addData("robot state", robotState);
            telemetry.addData("slide state", slideState);
            telemetry.addData("wrist state", wristState);
            telemetry.addData("loop time", deltaTime);
            telemetry.update();
        }
    }
    public void setRobotState(ROBOTSTATE targetRobotState) {
        lastRobotState = robotState;
        robotState = targetRobotState;
    }

    public void setSubstates(SLIDESTATE slideState, WRISTSTATE wristState, double driveSpeed) {
        this.slideState = slideState;
        this.wristState = wristState;
        this.driveSpeed = driveSpeed;
    }

    public void setNumOfGamepads() {
        duoDrive = !(gamepad2.getGamepadId() == -1);
    }

    public void initStates() {
        robotState = ROBOTSTATE.moving;
        lastRobotState = ROBOTSTATE.moving;
        slideState = SLIDESTATE.zero;
        wristState = WRISTSTATE.intake;
        wheelState = WHEELSTATE.stop;
    }

    public void initRobot() {
        robot.resetSliders();
        robot.autoWrist(Constants.wristMovingPos);
        robot.setWheelPowers(0);
    }
}

package org.firstinspires.ftc.teamcode.BluCru.opmode;

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
        maneuvering
    }

    ROBOTSTATE robotState, lastRobotState;

    Gamepad currentGamepad1, currentGamepad2, lastGamepad1, lastGamepad2;

    double slideZeroTime;
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
            lastGamepad1.copy(currentGamepad1);
            lastGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            robot.telemetry(telemetry);
            telemetry.update();
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
        robotState = ROBOTSTATE.maneuvering;
    }

    public void initRobot() {
        robot.resetSliders();
        robot.autoWrist(Constants.wristUpPos);
    }
}

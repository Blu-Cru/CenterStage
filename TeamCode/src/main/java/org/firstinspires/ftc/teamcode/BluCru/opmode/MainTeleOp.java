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
        moving, intake, eject, preOuttake, outtake, preHang, hanging
    }

    enum SLIDESTATE {
        zero, low, med, high, manual
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
    double vertControl, horzControl, rotateControl;
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

            // robot superstate control
            switch (robotState) {
                case moving:
                    setSubstates(SLIDESTATE.zero, WRISTSTATE.moving, WHEELSTATE.stop, Constants.driveSpeedMoving);
                    if(currentGamepad1.left_bumper && !currentGamepad1.right_bumper) {
                        setRobotState(ROBOTSTATE.intake);
                    } else if (currentGamepad1.right_bumper && !currentGamepad1.left_bumper) {
                        setRobotState(ROBOTSTATE.eject);
                    }

                    break;
                case intake:
                    setSubstates(SLIDESTATE.zero, WRISTSTATE.intake, WHEELSTATE.intake, Constants.driveSpeedIntake);
                    if(!currentGamepad1.left_bumper && lastGamepad1.right_bumper) {
                        setRobotState(ROBOTSTATE.moving);
                    }
                    break;
                case eject:
                    setSubstates(SLIDESTATE.zero, WRISTSTATE.intake, WHEELSTATE.outtake, Constants.driveSpeedEject);
                    if(!currentGamepad1.right_bumper && lastGamepad1.right_bumper) {
                        setRobotState(ROBOTSTATE.moving);
                    }
                    break;
                case preOuttake:
            }

            // loop time: current time - time at start of loop
            deltaTime = totalTimer.milliseconds() - lastTime;

            robot.telemetry(telemetry);
            telemetry.addData("loop time", deltaTime);
            telemetry.update();
        }
    }
    public void setRobotState(ROBOTSTATE targetRobotState) {
        lastRobotState = robotState;
        robotState = targetRobotState;
    }

    public void setSubstates(SLIDESTATE slideState, WRISTSTATE wristState, WHEELSTATE wheelState, double driveSpeed) {
        this.slideState = slideState;
        this.wristState = wristState;
        this.wheelState = wheelState;
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

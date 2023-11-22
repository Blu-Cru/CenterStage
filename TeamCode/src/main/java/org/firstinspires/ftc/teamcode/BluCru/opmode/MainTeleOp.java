package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BluCru.states.TeleOpStateMachine;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Robot;


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
    Robot robot;
    TeleOpStateMachine teleOpStateMachine;
    ElapsedTime totalTimer;
    double lastTime, deltaTime;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap);
        teleOpStateMachine = new TeleOpStateMachine(robot);
        robot.init();
        telemetry.addData("Status", "Initialized");

        waitForStart();

//        headingOffset = robot.getRawExternalHeading() + Math.toRadians(180);
        totalTimer = new ElapsedTime();

        while(opModeIsActive()) {
            lastTime = totalTimer.milliseconds();
            teleOpStateMachine.updateStates(gamepad1, gamepad2);

            // loop time: current time - time at start of loop
            deltaTime = totalTimer.milliseconds() - lastTime;

            teleOpStateMachine.telemetry(telemetry);
            telemetry.addData("loop time", deltaTime);
            telemetry.update();
        }
    }
}

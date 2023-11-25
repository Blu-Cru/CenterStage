package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BluCru.states.Alliance;
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
    Alliance alliance = Alliance.BLUE;
    Gamepad lastGamepad1 = new Gamepad();
    Gamepad lastGamepad2 = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap);
        teleOpStateMachine = new TeleOpStateMachine(robot);
        robot.init();

        while(opModeInInit()) {
            if(gamepad1.a && !lastGamepad1.a) {
                if(alliance == Alliance.BLUE) alliance = Alliance.RED;
                else alliance = Alliance.BLUE;
            }

            lastGamepad1.copy(gamepad1);
            lastGamepad2.copy(gamepad2);

            telemetry.addData("PICK UP UR CONTROLELRS", "");
            telemetry.addData("press a to change alliance (mantadory or else kick from team)", "");
            telemetry.addData("alliance: ", alliance);
            telemetry.update();
        }

        waitForStart();

        teleOpStateMachine.alliance = alliance;

//        headingOffset = robot.getRawExternalHeading() + Math.toRadians(180);
        totalTimer = new ElapsedTime();

        while(opModeIsActive()) {
            lastTime = totalTimer.milliseconds();
            teleOpStateMachine.updateStates(gamepad1, gamepad2);
            teleOpStateMachine.updateRobot(gamepad1, gamepad2);

            // loop time: current time - time at start of loop
            deltaTime = totalTimer.milliseconds() - lastTime;

            teleOpStateMachine.telemetry(telemetry);
            telemetry.addData("loop time", deltaTime);
            telemetry.update();
        }
    }
}

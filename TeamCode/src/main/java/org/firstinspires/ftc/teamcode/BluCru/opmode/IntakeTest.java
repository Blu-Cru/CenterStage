package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BluCru.Constants;
import org.firstinspires.ftc.teamcode.BluCru.Hardware6417;

@TeleOp(name = "intake test", group = "TeleOp")
public class IntakeTest extends LinearOpMode {
    enum ROBOTSTATE {
        moving, intake
    }
    Hardware6417 robot;
    Gamepad currentGamepad1, currentGamepad2, lastGamepad1, lastGamepad2;
    double vert, horz, rotate, driveSpeed, heading;
    double wheelPower;
    ROBOTSTATE robotState;
    ElapsedTime totalTimer;

    @Override
    public void runOpMode() throws InterruptedException {
        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        lastGamepad1 = new Gamepad();
        lastGamepad2 = new Gamepad();
        robot = new Hardware6417(hardwareMap);
        robot.initSlides();
        robot.initWrist();
        robot.initDrive(new Pose2d(0,0,0));
        robot.initWheels();
        robotState = ROBOTSTATE.moving;
        robot.autoWrist(Constants.wristMovingPos);
        driveSpeed = 0.5;

        waitForStart();
        totalTimer = new ElapsedTime();

        while(opModeIsActive()) {
            lastGamepad1.copy(currentGamepad1);
            lastGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if(gamepad1.left_trigger > Constants.triggerSens) {
                robotState = ROBOTSTATE.intake;
                wheelPower = Constants.wheelIntakePower;
            } else if(gamepad1.right_trigger > Constants.triggerSens) {
                robotState = ROBOTSTATE.intake;
                wheelPower = Constants.wheelOuttakePower;
            } else {
                robotState = ROBOTSTATE.moving;
                wheelPower = Constants.wheelStopPower;
            }

            switch(robotState) {
                case moving:
                    robot.autoSlider(Constants.sliderBasePos);
                    robot.autoWrist(Constants.wristMovingPos);
                    break;
                case intake:
                    robot.autoSlider(Constants.sliderIntakePos);
                    robot.stopWrist();
                    break;
            }

            robot.setWheelPowers(wheelPower);

            horz = Math.pow(gamepad1.left_stick_x, 3);
            vert = -Math.pow(gamepad1.left_stick_y, 3);
            rotate = -Math.pow(gamepad1.right_stick_x, 3);

            robot.holonomicDrive(horz, vert, rotate, driveSpeed, heading);

            telemetry.update();
        }
    }
}

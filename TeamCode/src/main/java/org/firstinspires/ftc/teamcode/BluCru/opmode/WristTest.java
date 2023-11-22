package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BluCru.Constants;
import org.firstinspires.ftc.teamcode.BluCru.Hardware6417;

@TeleOp(name = "wrist test", group = "TeleOp")
public class WristTest extends LinearOpMode {
    enum ROBOTSTATE {
        moving, intake, outtake
    }

    enum SLIDESTATE {
        zero, intake,outtake, low, med, high, manual
    }
    private Hardware6417 robot;
    private int target = 0;
    boolean goDown = false;

    Gamepad currentGamepad1, currentGamepad2, lastGamepad1, lastGamepad2;

    SLIDESTATE slideState = SLIDESTATE.zero;

    ElapsedTime totalTimer;
    double vert, horz, rotate, driveSpeed, heading;
    double wheelPower;


    double pos = 0;
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
        driveSpeed = 0.5;
        wheelPower = 0;



        double slideZeroTime = 0;
        robot.autoWrist(Constants.outtakeWristRetractPos);
        waitForStart();
        totalTimer = new ElapsedTime();
        while(opModeIsActive()) {
            lastGamepad1.copy(currentGamepad1);
            lastGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            horz = Math.pow(gamepad1.left_stick_x, 3);
            vert = -Math.pow(gamepad1.left_stick_y, 3);
            rotate = -Math.pow(gamepad1.right_stick_x, 3);

            if(gamepad1.a) {
                slideZeroTime = totalTimer.milliseconds();
                if(slideState != SLIDESTATE.zero) {
                    goDown = true;
                }
            }
            if(gamepad1.b) {
                slideState = SLIDESTATE.low;
            }
            if(gamepad1.x) {
                slideState = SLIDESTATE.med;
            }
            if(gamepad1.y) {
                slideState = SLIDESTATE.high;
            }

// wheels control
            if(gamepad1.left_trigger > Constants.triggerSens) {
                robot.setWheelPowers(Constants.outtakeServoIntakePower);
            }  else if(gamepad1.right_trigger > Constants.triggerSens) {
                robot.setWheelPowers(Constants.outtakeServoOuttakePower);
            } else {
                robot.setWheelPowers(0);
            }

            // slide control
            switch(slideState) {
                case zero:
                    if(totalTimer.milliseconds() - slideZeroTime > Constants.slideStallDelay) {
                        robot.resetSliders();
                    } else {
                        robot.autoSlider(Constants.sliderRetractPos);
                    }

                    if(robot.slider.getCurrentPosition() < Constants.sliderIntakeDelta) {
                        if(gamepad1.left_trigger > Constants.triggerSens) {
                            slideState = SLIDESTATE.intake;
                        }
                        if(gamepad1.right_trigger > Constants.triggerSens) {
                            slideState = SLIDESTATE.outtake;
                        }
                    }

                    robot.autoWrist(Constants.outtakeWristRetractPos);
                    break;
                case intake:
                    robot.autoSlider(Constants.sliderIntakePos);
                    robot.stopWrist();
                    if(!(gamepad1.left_trigger > Constants.triggerSens)) {
                        slideState = SLIDESTATE.zero;
                    }
                    break;
                case outtake:
                    robot.autoSlider(Constants.sliderIntakePos);
                    robot.stopWrist();
                    if(!(gamepad1.right_trigger > Constants.triggerSens)) {
                        slideState = SLIDESTATE.zero;
                    }
                    break;
                case low:
                    robot.autoSlider(Constants.sliderLowPos);
                    if(goDown) {
                        robot.autoWrist(Constants.outtakeWristOuttakePos);
                    } else if(robot.slider.getCurrentPosition() > Constants.sliderWristClearPos) {
                        robot.autoWrist(Constants.outtakeWristOuttakePos);
                    } else {
                        robot.autoWrist(Constants.outtakeWristRetractPos);
                    }
                    if(totalTimer.milliseconds() - slideZeroTime > Constants.slideDownDelay && goDown) {
                        slideState = slideState.zero;
                        goDown = false;
                    }
                    break;
                case med:
                    robot.autoSlider(Constants.sliderMedPos);
                    if(goDown) {
                        robot.autoWrist(Constants.outtakeWristRetractPos);
                    } else if(robot.slider.getCurrentPosition() > Constants.sliderWristClearPos) {
                        robot.autoWrist(Constants.outtakeWristOuttakePos);
                    } else {
                        robot.autoWrist(Constants.outtakeWristRetractPos);
                    }
                    if(totalTimer.milliseconds() - slideZeroTime > Constants.slideDownDelay && goDown) {
                        slideState = slideState.zero;
                        goDown = false;
                    }
                    break;
                case high:
                    robot.autoSlider(Constants.sliderHighPos);
                    if(goDown) {
                        robot.autoWrist(Constants.outtakeWristRetractPos);
                    } else if(robot.slider.getCurrentPosition() > Constants.sliderWristClearPos) {
                        robot.autoWrist(Constants.outtakeWristOuttakePos);
                    } else {
                        robot.autoWrist(Constants.outtakeWristRetractPos);
                    }
                    if(totalTimer.milliseconds() - slideZeroTime > Constants.slideDownDelay && goDown) {
                        slideState = slideState.zero;
                        goDown = false;
                    }
                    break;
                case manual:
                    break;
            }

            telemetry.addData("go down", goDown);
            telemetry.addData("slide state", slideState);
            telemetry.addData("wrist position", robot.wrist.getPosition());
            telemetry.addData("time since slide 0", totalTimer.milliseconds() - slideZeroTime);
            telemetry.addData("current slider position", robot.slider.getCurrentPosition());
            telemetry.addData("power", robot.slider.getPower());
            telemetry.addData("saved pos", pos);
            telemetry.update();
        }
    }
}

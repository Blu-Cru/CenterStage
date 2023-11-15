package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BluCru.Constants;
import org.firstinspires.ftc.teamcode.BluCru.Hardware6417;


@TeleOp(name = "dual slide test", group = "TeleOp")
public class DualSlideTest extends LinearOpMode {
    enum SLIDESTATE {
        zero, low, med, high, manual
    }
    private Hardware6417 robot;
    private int target = 0;

    Gamepad currentGamepad1, currentGamepad2, lastGamepad1, lastGamepad2;

    SLIDESTATE slideState = SLIDESTATE.zero;

    ElapsedTime totalTimer;


    double pos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        lastGamepad1 = new Gamepad();
        lastGamepad2 = new Gamepad();
        robot = new Hardware6417(hardwareMap);
        robot.initSlides();


        double slideZeroTime = 0;

        waitForStart();
        totalTimer = new ElapsedTime();
        while(opModeIsActive()) {
            lastGamepad1.copy(currentGamepad1);
            lastGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if(gamepad1.a) {
                slideState = SLIDESTATE.zero;
                if(!lastGamepad1.a) {
                    slideZeroTime = totalTimer.milliseconds();
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

            // slide control
            switch(slideState) {
                case zero:
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

            telemetry.addData("time since slide 0", totalTimer.milliseconds() - slideZeroTime);
            telemetry.addData("current", robot.slider.getCurrentPosition());
            telemetry.addData("power", robot.slider.getPower());
            telemetry.addData("saved pos", pos);
            telemetry.update();
        }
    }
}

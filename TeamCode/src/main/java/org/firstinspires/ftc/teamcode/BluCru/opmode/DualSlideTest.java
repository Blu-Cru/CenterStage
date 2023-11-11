package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "dual slide test", group = "TeleOp")
public class DualSlideTest extends LinearOpMode {
    DcMotorEx slider, auxSlider;

    double pos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        slider = hardwareMap.get(DcMotorEx.class, "slider");
        auxSlider = hardwareMap.get(DcMotorEx.class, "auxSlider");

        resetSliders();

        slider.setDirection(DcMotorSimple.Direction.FORWARD);
        slider.setPower(0);
        auxSlider.setDirection(DcMotorSimple.Direction.REVERSE);
        auxSlider.setPower(0);


        boolean lastLB1 = false;
        boolean lastRB1 = false;
        int delta = 100;
        int pos = 0;
        double vert;

        waitForStart();
        while(opModeIsActive()) {
            vert = -gamepad1.left_stick_y;

            delta = (int)(-gamepad1.right_stick_y*200) + 200;
            if(gamepad1.right_bumper && !lastRB1) {
                slider.setTargetPosition(slider.getCurrentPosition() + 100);
            }
            if(gamepad1.left_bumper && !lastLB1) {
                slider.setTargetPosition(slider.getCurrentPosition() - 100);
            }
            if(gamepad1.a) {
                pos = slider.getCurrentPosition();
            }
            if(Math.abs(vert) > 0.1) {
                slider.setPower(vert);
                auxSlider.setPower(vert);
            } else {
                slider.setPower(0);
                auxSlider.setPower(0);
            }

            lastLB1 = gamepad1.left_bumper;
            lastRB1 = gamepad1.right_bumper;

            telemetry.addData("target", slider.getTargetPosition());
            telemetry.addData("current", auxSlider.getCurrentPosition());
            telemetry.addData("delta", delta);
            telemetry.addData("power", slider.getPower());
            telemetry.addData("saved pos", pos);
            telemetry.update();
        }
    }
    public void resetSliders() {
        slider.setPower(0);
        slider.setTargetPosition(0);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        auxSlider.setPower(0);
        auxSlider.setTargetPosition(0);
        auxSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        auxSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        auxSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}

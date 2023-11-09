package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "dual slide test", group = "TeleOp")
public class DualSlideTest extends LinearOpMode {

    double pos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx slider = hardwareMap.get(DcMotorEx.class, "slider");
        DcMotorEx auxSlider = hardwareMap.get(DcMotorEx.class, "auxSlider");

        slider.setDirection(DcMotorSimple.Direction.REVERSE);
        slider.setPower(0);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        auxSlider.setDirection(DcMotorSimple.Direction.FORWARD);
        auxSlider.setPower(0);
        auxSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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
            telemetry.addData("current", slider.getCurrentPosition());
            telemetry.addData("delta", delta);
            telemetry.addData("power", slider.getPower());
            telemetry.addData("saved pos", pos);
            telemetry.update();
        }
    }
}

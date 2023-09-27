package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Neil Intake Test", group = "Neil")
public class NeilIntakeTest extends LinearOpMode {
    double power = 0.5;
    CRServo left;
    CRServo right;
    @Override
    public void runOpMode() throws InterruptedException {
        right = hardwareMap.get(CRServo.class, "right");
        left = hardwareMap.get(CRServo.class, "left");
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a) {
                power = 0.5;
            }
            if(gamepad1.b) {
                power = 1;
            }
            if(gamepad1.x) {
                power = 0;
            }
            if(gamepad1.y){
                power = -1;
            }
            left.setPower(power);
            right.setPower(-power);

            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }
    }
}

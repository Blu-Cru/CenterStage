package org.firstinspires.ftc.teamcode.BluCru.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.BluCru.Constants;
import org.firstinspires.ftc.teamcode.BluCru.states.LiftState;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Intake;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Lift;

@TeleOp(name = "lift test", group = "TeleOp")
public class LiftTest extends LinearOpMode {
    Lift lift;
    Intake intake;
    Gamepad lastGamepad1;
    Gamepad lastGamepad2;

    enum LIFTSTATE {
        ZERO,
        LOW,
        MED,
        HIGH
    }

    LIFTSTATE liftState = LIFTSTATE.ZERO;
    @Override
    public void runOpMode() throws InterruptedException {
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        lastGamepad1 = new Gamepad();
        lastGamepad2 = new Gamepad();

        lift.init();
        lift.liftState = LiftState.AUTO;
        intake.init();

        waitForStart();
        while(opModeIsActive()) {
            intake.setOuttakeWristPosition(Constants.outtakeWristRetractPos);
            if(gamepad1.a) {
                lift.setMotionProfileTargetPosition(0);
            }
            if(gamepad1.b && !lastGamepad1.b) {
                lift.setMotionProfileTargetPosition(Constants.sliderLowPos);
            }
            if(gamepad1.x && !lastGamepad1.x) {
                lift.setMotionProfileTargetPosition(Constants.sliderMedPos);
            }
            if(gamepad1.y && !lastGamepad1.y) {
                lift.setMotionProfileTargetPosition(Constants.sliderHighPos);
            }


            lastGamepad1.copy(gamepad1);
            lastGamepad2.copy(gamepad2);

            lift.update();

            lift.telemetry(telemetry);
            lift.motionProfileTelemetry(telemetry);
            telemetry.update();
        }
    }
}

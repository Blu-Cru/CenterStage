package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BluCru.Constants;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Intake;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Lift;

@TeleOp(name = "lift test", group = "TeleOp")
public class LiftTest extends LinearOpMode {
    Lift lift;
    Intake intake;

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
        lift.init();
        intake.init();
        waitForStart();
        while(opModeIsActive()) {
            intake.setOuttakeWristPosition(Constants.outtakeWristRetractPos);
            if(gamepad1.a) {
                lift.resetSliderStallTimer();
                liftState = LIFTSTATE.ZERO;
            }
            if(gamepad1.b) {
                liftState = LIFTSTATE.LOW;
            }
            if(gamepad1.x) {
                liftState = LIFTSTATE.MED;
            }
            if(gamepad1.y) {
                liftState = LIFTSTATE.HIGH;
            }

            switch(liftState) {
                case ZERO:
                    lift.setTargetPos(Constants.sliderRetractPos);
                    break;
                case LOW:
                    lift.setTargetPos(Constants.sliderLowPos);
                    break;
                case MED:
                    lift.setTargetPos(Constants.sliderMedPos);
                    break;
                case HIGH:
                    lift.setTargetPos(Constants.sliderHighPos);
                    break;
            }

            lift.update();

            telemetry.addData("target", lift.getTargetPos());
            telemetry.addData("current", lift.getCurrentPos());
            telemetry.addData("liftState", liftState);
            telemetry.update();
        }
    }
}

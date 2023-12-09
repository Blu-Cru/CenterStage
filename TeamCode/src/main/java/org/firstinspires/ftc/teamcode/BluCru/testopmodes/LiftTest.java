package org.firstinspires.ftc.teamcode.BluCru.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BluCru.Constants;
import org.firstinspires.ftc.teamcode.BluCru.states.LiftState;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Intake;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Lift;

@TeleOp(name = "lift test", group = "TeleOp")
public class LiftTest extends LinearOpMode {
    Lift lift;
    Intake intake;

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
                lift.resetLiftStallTimer();
                lift.liftState = LiftState.RETRACT;
            }
            if(gamepad1.b) {
                lift.setTargetPos(Constants.sliderLowPos);
                lift.liftState = LiftState.AUTO;
            }
            if(gamepad1.x) {
                lift.setTargetPos(Constants.sliderMedPos);
                lift.liftState = LiftState.AUTO;
            }
            if(gamepad1.y) {
                lift.setTargetPos(Constants.sliderHighPos);
                lift.liftState = LiftState.AUTO;
            }


            lift.update();

            lift.telemetry(telemetry);
            telemetry.update();
        }
    }
}

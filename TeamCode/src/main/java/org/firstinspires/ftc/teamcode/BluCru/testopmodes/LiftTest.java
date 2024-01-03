package org.firstinspires.ftc.teamcode.BluCru.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.BluCru.Constants;
import org.firstinspires.ftc.teamcode.BluCru.states.LiftState;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Intake;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Lift;

@Config
@TeleOp(name = "lift test", group = "TeleOp")
public class LiftTest extends LinearOpMode {
    public static double maxVelocity = 1000;
    public static double maxAcceleration = 2000;

    Lift lift;
    Intake intake;
    Gamepad lastGamepad1;
    Gamepad lastGamepad2;

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
            lift.setMotionProfileConstraints(maxVelocity, maxAcceleration);

            intake.setOuttakeWristPosition(Constants.outtakeWristRetractPos);

            if(gamepad1.a && !lastGamepad1.a) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> lift.setMotionProfileTargetPosition(0))
                        )
                );
            }

            if(gamepad1.b && !lastGamepad1.b) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> lift.setMotionProfileTargetPosition(Constants.sliderLowPos)),
                                new WaitCommand(3500),
                                new InstantCommand(() -> lift.setMotionProfileTargetPosition(0))
                        )
                );
            }

            if(gamepad1.x && !lastGamepad1.x) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> lift.setMotionProfileTargetPosition(Constants.sliderMedPos))
                        )
                );
            }

            if(gamepad1.y && !lastGamepad1.y) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> lift.setMotionProfileTargetPosition(Constants.sliderHighPos))
                        )
                );
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
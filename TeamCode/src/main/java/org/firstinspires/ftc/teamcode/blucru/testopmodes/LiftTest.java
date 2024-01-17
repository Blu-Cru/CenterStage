package org.firstinspires.ftc.teamcode.blucru.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.blucru.Constants;
import org.firstinspires.ftc.teamcode.blucru.states.LiftState;
import org.firstinspires.ftc.teamcode.blucru.subsystems.Intake;
import org.firstinspires.ftc.teamcode.blucru.subsystems.Lift;

@Config
@TeleOp(name = "lift test", group = "TeleOp")
public class LiftTest extends LinearOpMode {
    public static double maxVelocity = 13000;
    public static double maxAcceleration = 20000;

    Lift lift;
    Intake intake;
    Gamepad lastGamepad1;
    Gamepad lastGamepad2;

    @Override
    public void runOpMode() throws InterruptedException {
        lift = new Lift(hardwareMap);
        intake = new Intake(hardwareMap);
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
                                new WaitCommand(100),
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

            CommandScheduler.getInstance().run();

            lastGamepad1.copy(gamepad1);
            lastGamepad2.copy(gamepad2);

            lift.write();

            lift.telemetry(telemetry);
            lift.motionProfileTelemetry(telemetry);
            telemetry.update();
        }
    }
}
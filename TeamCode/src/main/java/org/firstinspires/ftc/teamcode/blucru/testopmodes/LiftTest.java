package org.firstinspires.ftc.teamcode.blucru.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.blucru.states.LiftState;
import org.firstinspires.ftc.teamcode.blucru.subsystems.Lift;
import org.firstinspires.ftc.teamcode.blucru.subsystems.MotionProfile;

@Config
@TeleOp(name = "lift test", group = "TeleOp")
public class LiftTest extends LinearOpMode {
    public static double maxVelocity = 400.0;
    public static double maxAcceleration = 300.0;
    public static int xI = 500;
    public static int xTarget = 200;
    public static double vI = -60;
    public static double vMax = 500;
    public static double aMax = 200;
    public static int run = 0;
    int lastRun = run;

    Lift lift;
    Gamepad lastGamepad1;
    Gamepad lastGamepad2;

    @Override
    public void runOpMode() throws InterruptedException {
        lift = new Lift(hardwareMap);
        lastGamepad1 = new Gamepad();
        lastGamepad2 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift.init();
        lift.liftState = LiftState.AUTO;
//        intake.init();

        waitForStart();
        while(opModeIsActive()) {
            lift.read();

            if(run != lastRun) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> lift.setMotionProfile(new MotionProfile(xTarget, xI, vI, vMax, aMax)))
                        )
                );
            }
//            lift.setMotionProfileConstraints(maxVelocity, maxAcceleration);

//            intake.setOuttakeWristPosition(Constants.outtakeWristRetractPos);

            if(gamepad1.a && !lastGamepad1.a) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> lift.setMotionProfileTargetPos(-10))
                        )
                );
            }

            if(gamepad1.b && !lastGamepad1.b) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> lift.setMotionProfileTargetPos(1000))
                        )
                );
            }

            if(gamepad1.x && !lastGamepad1.x) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> lift.setMotionProfileTargetPos(1500))
                        )
                );
            }

            if(gamepad1.y && !lastGamepad1.y) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> lift.setMotionProfileTargetPos(2000))
                        )
                );
            }

            if(Math.abs(gamepad1.left_stick_y) > 0.1) {
                lift.liftState = LiftState.MANUAL;
                lift.power = -gamepad1.left_stick_y;
            }
            if(!(Math.abs(gamepad1.left_stick_y) > 0.1) && Math.abs(lastGamepad1.left_stick_y) > 0.1) {
                lift.liftState = LiftState.AUTO;
            }

            write();
        }
    }

    public void read() {
        lift.read();
    }

    public void write() {
        lastRun = run;
        CommandScheduler.getInstance().run();
        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);
        lift.write();
        lift.telemetry(telemetry);
        lift.motionProfileTelemetry(telemetry);
        telemetry.update();
    }
}
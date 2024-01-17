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
import org.firstinspires.ftc.teamcode.blucru.subsystems.MotionProfiler;

@Config
@TeleOp(name = "lift test", group = "TeleOp")
public class LiftTest extends LinearOpMode {
    public static double maxVelocity = 400.0;
    public static double maxAcceleration = 300.0;
    public static double testVelocity = 400.0;
    public static double testAccel = 400.0;
    public static int testInitial = 0;
    public static int testFinal = 1000;
    public static double testVI = -400.0;
    public static int run = 0;

    Lift lift;
//    Intake intake;
    Gamepad lastGamepad1;
    Gamepad lastGamepad2;
    private int lastRun = run;

    @Override
    public void runOpMode() throws InterruptedException {
        lift = new Lift(hardwareMap, telemetry);
//        intake = new Intake(hardwareMap, telemetry);
        lastGamepad1 = new Gamepad();
        lastGamepad2 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift.init();
        lift.liftState = LiftState.AUTO;
//        intake.init();

        waitForStart();
        while(opModeIsActive()) {
//            lift.setMotionProfileConstraints(maxVelocity, maxAcceleration);

//            intake.setOuttakeWristPosition(Constants.outtakeWristRetractPos);

            if(gamepad1.a && !lastGamepad1.a) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> lift.setMotionProfileTargetPosition(0))
                        )
                );
            }

            if(run != lastRun || (gamepad1.b && !lastGamepad1.b)) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> lift.setMotionProfiler(new MotionProfiler(testFinal, testInitial, testVI, testVelocity, testAccel)))
//                                new WaitCommand(2000),
//                                new InstantCommand(() -> lift.setMotionProfileTargetPosition(0))
                        )
                );
            }

            if(gamepad1.x && !lastGamepad1.x) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> lift.setMotionProfiler(new MotionProfiler(0, 1000, -500, maxVelocity, maxAcceleration)))
                        )
                );
            }

            if(gamepad1.y && !lastGamepad1.y) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> lift.setMotionProfiler(new MotionProfiler(1000, 0, 0, maxVelocity, maxAcceleration)))
                        )
                );
            }

            if(Math.abs(gamepad1.left_stick_y) > 0.1) {
                lift.liftState = LiftState.MANUAL;
                lift.power = -gamepad1.left_stick_y;
            }
            if(!(Math.abs(gamepad1.left_stick_y) > 0.1) && Math.abs(lastGamepad1.left_stick_y) > 0.1) {
                lift.liftState = LiftState.AUTO;
                lift.setMotionProfileTargetPosition(lift.inverseP(lift.power));
            }

            lastRun = run;
            CommandScheduler.getInstance().run();

            lastGamepad1.copy(gamepad1);
            lastGamepad2.copy(gamepad2);

            lift.update();

            lift.telemetry(telemetry);
            lift.motionProfileTelemetry(telemetry);
            telemetry.update();
        }
    }
}
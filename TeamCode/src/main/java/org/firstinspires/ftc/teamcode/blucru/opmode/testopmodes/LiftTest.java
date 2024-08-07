package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(name = "lift test", group = "test")
public class LiftTest extends BluLinearOpMode {
    public static double maxVelocity = 10000.0;
    public static double maxAcceleration = 7000.0;
    public static int xI = 500;
    public static int xTarget = 200;
    public static double vI = -60;
    public static double vMax = 500;
    public static double aMax = 200;
//    public static int run = 0;
//    int lastRun = run;

    boolean lastB = false;
    boolean lastA = false;
    boolean lastX = false;
    boolean lastY = false;

    public void initialize() {
        addLift();
        enableFTCDashboard();
    }

    public void periodic() {
        lift.updatePID();

        if(gamepad1.a && !lastA) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> lift.setMotionProfileTargetPos(0))
                    )
            );
        }
        lastA = gamepad1.a;

        if(gamepad1.b && !lastB) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> lift.setMotionProfileTargetPos(500))
                    )
            );
        }
        lastB = gamepad1.b;

        if(gamepad1.x && !lastX) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> lift.setTargetPos(1000))
                    )
            );
        }
        lastX = gamepad1.x;

        if(gamepad1.y && !lastY) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> lift.setTargetPos(1500))
                    )
            );
        }
        lastY = gamepad1.y;

//        if(Math.abs(gamepad1.left_stick_y) > 0.1) {
//            lift.liftState = LiftState.MANUAL;
//            outtake.setManualSlidePower(-gamepad1.left_stick_y);
//        }
//        if(!(Math.abs(gamepad1.left_stick_y) > 0.1) && Math.abs(lastGamepad1.left_stick_y) > 0.1) {
//            lift.liftState = LiftState.PID;
//        }


//        lastRun = run;
        CommandScheduler.getInstance().run();
    }

    public void telemetry() {
        lift.testTelemetry(telemetry);
        lift.motionProfileTelemetry(telemetry);
//        outtake.testTelemetry(telemetry);
    }
}
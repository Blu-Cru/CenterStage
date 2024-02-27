package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.blucru.common.states.LiftState;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

@Config
@TeleOp(name = "lift test", group = "test")
public class LiftTest extends LinearOpMode {
    public static double maxVelocity = 10000.0;
    public static double maxAcceleration = 7000.0;
    public static int xI = 500;
    public static int xTarget = 200;
    public static double vI = -60;
    public static double vMax = 500;
    public static double aMax = 200;
    public static int run = 0;
    int lastRun = run;

    Outtake outtake;
    Gamepad lastGamepad1;
    Gamepad lastGamepad2;

    boolean lastB = false;
    boolean lastA = false;
    boolean lastX = false;
    boolean lastY = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Outtake outtake = robot.addOuttake();
        lastGamepad1 = new Gamepad();
        lastGamepad2 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init();
        outtake.lift.liftState = LiftState.AUTO;

        waitForStart();
        while(opModeIsActive()) {
            robot.read();

//            if(run != lastRun) {
//                CommandScheduler.getInstance().schedule(
//                        new SequentialCommandGroup(
//                                new InstantCommand(() -> lift.setMotionProfile(new MotionProfile(xTarget, xI, vI, vMax, aMax)))
//                        )
//                );
//            }
//            outtake.lift.setMotionProfileConstraints(maxVelocity, maxAcceleration);

//            intake.setOuttakeWristPosition(Constants.outtakeWristRetractPos);

            if(gamepad1.a && !lastA) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> outtake.lift.setMotionProfileTargetPos(0))
                        )
                );
            }
            lastA = gamepad1.a;

            if(gamepad1.b && !lastB) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> outtake.lift.setMotionProfileTargetPos(500))
                        )
                );
            }
            lastB = gamepad1.b;

            if(gamepad1.x && !lastGamepad1.x) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> outtake.lift.setTargetPos(1000))
                        )
                );
            }

            if(gamepad1.y && !lastGamepad1.y) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> outtake.lift.setTargetPos(1500))
                        )
                );
            }

            if(Math.abs(gamepad1.left_stick_y) > 0.1) {
                outtake.lift.liftState = LiftState.MANUAL;
                outtake.lift.power = -gamepad1.left_stick_y;
            }
            if(!(Math.abs(gamepad1.left_stick_y) > 0.1) && Math.abs(lastGamepad1.left_stick_y) > 0.1) {
                outtake.lift.liftState = LiftState.AUTO;
            }

            robot.write();

            lastRun = run;
            CommandScheduler.getInstance().run();
            lastGamepad1.copy(gamepad1);
            lastGamepad2.copy(gamepad2);
            robot.telemetry(telemetry);
            outtake.lift.motionProfileTelemetry(telemetry);
            telemetry.update();
        }
    }
}
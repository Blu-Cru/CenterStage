package org.firstinspires.ftc.teamcode.blucru.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.blucru.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.states.AutoState;
import org.firstinspires.ftc.teamcode.blucru.states.Side;
import org.firstinspires.ftc.teamcode.blucru.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.trajectories.Trajectories;
import org.firstinspires.ftc.teamcode.blucru.vision.CVMaster;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name ="Auto", group = "Auto")
public class CenterCycleAuto extends LinearOpMode {
    private Robot robot;
    private Alliance alliance = Alliance.RED;
    private Side side = Side.CLOSE;
    private Trajectories trajectories;
    private CVMaster cvMaster;
    private AutoState autoState = AutoState.INIT;

    Gamepad lastGamepad1;
    Gamepad lastGamepad2;

    TrajectorySequence placementFar;
    TrajectorySequence placementClose;
    TrajectorySequence placementCenter;

    TrajectorySequence depositFar;
    TrajectorySequence depositClose;
    TrajectorySequence depositCenter;

    TrajectorySequence stackSetup;
    TrajectorySequence stackGrab;
    TrajectorySequence deposit;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap);
        lastGamepad1 = new Gamepad();
        lastGamepad2 = new Gamepad();
        while(!isStopRequested() && opModeInInit()) {
            telemetry.addData("state: ", autoState);
            telemetry.addData("alliance: ", alliance);
            telemetry.addData("side: ", side);

            switch (autoState) {
                case INIT:
                    if(gamepad1.x && !lastGamepad1.x)
                        alliance = alliance == Alliance.RED ? Alliance.BLUE : Alliance.RED;

                    if(gamepad1.b && !lastGamepad1.b)
                        side = side == Side.CLOSE ? Side.FAR : Side.CLOSE;

                    if(gamepad1.a && !lastGamepad1.a) {
                        autoState = AutoState.BUILD;

                        // build trajectories
                        trajectories = new Trajectories(alliance, side);
                        placementFar = trajectories.placementFar(robot);
                        placementClose = trajectories.placementClose(robot);
                        placementCenter = trajectories.placementCenter(robot);
                        depositFar = trajectories.depositFar(robot);
                        depositClose = trajectories.depositClose(robot);
                        depositCenter = trajectories.depositCenter(robot);
//                        stackSetup = trajectories.stackSetup(robot);
//                        stackGrab = trajectories.stackGrab(robot);
//                        deposit = trajectories.deposit(robot);
                    }


                    telemetry.addData("Press x (square) to cycle alliance", "");
                    telemetry.addData("Press b (circle) to cycle side", "");
                    telemetry.addData("Press a (x) to build trajectories", "");
                    break;
                case BUILD:
                    autoState = AutoState.DETECTION;
                    break;
                case DETECTION:
            }
            lastGamepad1.copy(gamepad1);
            lastGamepad2.copy(gamepad2);
            telemetry.update();
        }

        waitForStart();
    }
}

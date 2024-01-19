package org.firstinspires.ftc.teamcode.blucru.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    double position = 1;

    ElapsedTime runtime;

    Gamepad lastGamepad1;
    Gamepad lastGamepad2;

    TrajectorySequence farAuto;
    TrajectorySequence closeAuto;
    TrajectorySequence centerAuto;

    TrajectorySequence auto;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap);
        lastGamepad1 = new Gamepad();
        lastGamepad2 = new Gamepad();

        robot.init();
        robot.purplePixelHolder.retracted = false;

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

                        farAuto = trajectories.farCenterCycle(robot);
                        closeAuto = trajectories.closeCenterCycle(robot);
                        centerAuto = trajectories.centerCenterCycle(robot);
                    }


                    telemetry.addData("Press x (square) to cycle alliance", "");
                    telemetry.addData("Press b (circle) to cycle side", "");
                    telemetry.addData("Press a (x) to build trajectories", "");
                    break;
                case BUILD:
                    autoState = AutoState.DETECTION;
                    break;
                case DETECTION:
                    cvMaster = new CVMaster(hardwareMap, alliance);
                    cvMaster.detectProp();

                    position = cvMaster.propDetector.position;

                    telemetry.addData("position", cvMaster.propDetector.position);
                    telemetry.addData("average 0", cvMaster.propDetector.average0);
                    telemetry.addData("average 1", cvMaster.propDetector.average1);
                    telemetry.addData("average 2", cvMaster.propDetector.average2);
                    break;
            }
            lastGamepad1.copy(gamepad1);
            lastGamepad2.copy(gamepad2);
            telemetry.update();
        }

        switch(alliance) {
            case RED:
                if(position == 0) {
                    auto = farAuto;
                } else if(position == 1) {
                    auto = centerAuto;
                } else if(position == 2) {
                    auto = closeAuto;
                }
                break;
            case BLUE:
                if(position == 0) {
                    auto = closeAuto;
                } else if(position == 1) {
                    auto = centerAuto;
                } else if(position == 2) {
                    auto = farAuto;
                }
                break;
        }

        waitForStart();

        autoState = AutoState.RUNNING;
        runtime = new ElapsedTime();
        cvMaster.stop();

        robot.drivetrain.setPoseEstimate(trajectories.getStartPose());
        robot.drivetrain.followTrajectorySequenceAsync(auto);

        while(!isStopRequested() && opModeIsActive()) {
            robot.read();

            switch(autoState) {
                case INIT:
                    break;
                case BUILD:
                    break;
                case DETECTION:
                    break;
                case RUNNING:
                    if(!robot.drivetrain.isBusy()) {
                        autoState = AutoState.STOP;
                    }
                    break;
                case STOP:
                    break;
            }

            robot.write();

            telemetry.addData("runtime", runtime.seconds());
            telemetry.addData("state: ", autoState);
            telemetry.addData("alliance: ", alliance);
            telemetry.addData("side: ", side);
            telemetry.addData("position", position);
            telemetry.update();
        }
    }
}

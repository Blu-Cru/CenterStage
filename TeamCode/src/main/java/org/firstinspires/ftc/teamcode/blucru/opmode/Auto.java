package org.firstinspires.ftc.teamcode.blucru.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.states.AutoState;
import org.firstinspires.ftc.teamcode.blucru.common.states.AutoType;
import org.firstinspires.ftc.teamcode.blucru.common.states.Initialization;
import org.firstinspires.ftc.teamcode.blucru.common.states.ParkType;
import org.firstinspires.ftc.teamcode.blucru.common.states.Side;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.PurplePixelHolder;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Trajectories;
import org.firstinspires.ftc.teamcode.blucru.common.vision.CVMaster;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

@Autonomous(name ="Auto", group = "Auto")
public class Auto extends LinearOpMode {
    ArrayList<TrajectorySequence> trajectoryList;
    ArrayList<TrajectorySequence> trajectoriesFar;
    ArrayList<TrajectorySequence> trajectoriesClose;
    ArrayList<TrajectorySequence> trajectoriesCenter;

    int trajIndex;

    Robot robot;
    Drivetrain drivetrain;
    Intake intake;
    Outtake outtake;
    PurplePixelHolder purplePixelHolder;

    private Alliance alliance = Alliance.RED;
    private Side side = Side.CLOSE;
    private AutoType autoType = AutoType.CENTER_CYCLE;
    private ParkType parkType = ParkType.CENTER;
    private Trajectories trajectories;
    private CVMaster cvMaster;
    private AutoState autoState = AutoState.INIT;
    int position = 1;

    ElapsedTime runtime;

    boolean lastA = false;
    boolean lastB = false;
    boolean lastX = false;
    boolean lastY = false;

    double dt;
    double lastTime = System.currentTimeMillis();
    int loop = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        drivetrain = robot.addDrivetrain(false);
        intake = robot.addIntake();
        outtake = robot.addOuttake();
        purplePixelHolder = robot.addPurplePixelHolder();


        robot.init();

        while(!isStopRequested() && opModeInInit()) {
            switch (autoState) {
                case INIT:
                    if(gamepad1.x && !lastX) {
                        alliance = alliance.flip();
                        Initialization.alliance = alliance;
                    }
                    lastX = gamepad1.x;
                    if(gamepad1.b && !lastB) side = side.flip();
                    lastB = gamepad1.b;
                    if(gamepad1.y && !lastY) parkType = parkType.cycle();
                    lastY = gamepad1.y;
                    if(gamepad1.a && !lastA) autoType = autoType.cycle();
                    lastA = gamepad1.a;

                    if(gamepad1.right_stick_button) {
                        autoState = AutoState.BUILD;

                        // build trajectories
                        telemetry.addData("Building trajectories . . . ", "");
                        telemetry.update();

                        trajectories = new Trajectories(alliance, side, autoType, parkType);
                        trajectoriesCenter = trajectories.buildTrajectoriesCenter(robot);
                        trajectoriesClose = trajectories.buildTrajectoriesClose(robot);
                        trajectoriesFar = trajectories.buildTrajectoriesFar(robot);
                    }

                    telemetry.addData("Press y (triangle) to cycle PARK: ", parkType);
                    telemetry.addData("Press a (cross) to cycle AUTO TYPE: ", autoType);
                    telemetry.addData("Press x (square) to cycle ALLIANCE: ", alliance);
                    telemetry.addData("Press b (circle) to cycle side. SIDE: ", side);
                    telemetry.addData("Press right stick button to build trajectories", "");
                    break;
                case BUILD:
                    cvMaster = robot.addCVMaster(alliance);
                    autoState = AutoState.DETECTION;
                    telemetry.addData("state: ", autoState);
                    telemetry.addData("auto type: ", autoType);
                    telemetry.addData("park: ", parkType);
                    telemetry.addData("alliance: ", alliance);
                    telemetry.addData("side: ", side);
                    break;
                case DETECTION:
                    cvMaster.detectProp();

                    position = cvMaster.propDetector.position;

                    telemetry.addData("state: ", autoState);
                    telemetry.addData("AUTO: ", autoType);
                    telemetry.addData("Park: ", parkType);
                    telemetry.addData("Alliance: ", alliance);
                    telemetry.addData("Side: ", side);
                    telemetry.addData("POSITION: ", cvMaster.propDetector.position);
                    telemetry.addData("average 0", cvMaster.propDetector.average0);
                    telemetry.addData("average 1", cvMaster.propDetector.average1);
                    telemetry.addData("average 2", cvMaster.propDetector.average2);
                    break;
            }
            telemetry.update();
        }

        switch(alliance) {
            case RED:
                if(position == 0) {
                    trajectoryList = trajectoriesFar;
                } else if(position == 1) {
                    trajectoryList = trajectoriesCenter;
                } else {
                    trajectoryList = trajectoriesClose;
                }
                break;
            case BLUE:
                if(position == 0) {
                    trajectoryList = trajectoriesClose;
                } else if(position == 1) {
                    trajectoryList = trajectoriesCenter;
                } else {
                    trajectoryList = trajectoriesFar;
                }
                break;
        }

        waitForStart();
        cvMaster.stop();

        autoState = AutoState.RUNNING;
        trajIndex = 0;
        runtime = new ElapsedTime();

        drivetrain.setPoseEstimate(trajectories.getStartPose());

        while(!isStopRequested() && opModeIsActive()) {
            robot.read();

            switch(autoState) {
                case RUNNING:


                    if(!drivetrain.isBusy()) {
                        if(trajIndex == trajectoryList.size()) {
                            autoState = AutoState.STOP;
                            break;
                        } else {
                            drivetrain.followTrajectorySequenceAsync(trajectoryList.get(trajIndex));
                            trajIndex++;
                        }
                    }

                    drivetrain.updateTrajectory();
                    break;
                case STOP:
                    drivetrain.setWeightedDrivePower(new Pose2d(0, 0, 0));
                    Pose2d pose;
                    if(alliance == Alliance.RED) {
                        pose = drivetrain.getPoseEstimate();
                    } else {
                        pose = new Pose2d(0, 0, drivetrain.getPoseEstimate().getHeading() + Math.PI);
                    }
                    Initialization.POSE = pose;
                    break;
            }

            // stop
            if(runtime.seconds() > 29.3) {
                autoState = AutoState.STOP;
                robot.outtake.retractWrist();
            }
            if(runtime.seconds() > 29.5) {
                robot.outtake.lift.setTargetPos(0);
                robot.outtake.lock();
            }

            robot.write();

            dt = runtime.milliseconds() - lastTime;
            lastTime = runtime.milliseconds();

            // do telemetry every 50 loops
            loop++;
            if(loop > 50) {
                if (!drivetrain.followerIsWithinTolerance()) autoState = AutoState.STOP;
                telemetry.addData("Running trajectory " + trajIndex + " out of ", trajectoryList.size());
                telemetry.addData("runtime", runtime.seconds());
                telemetry.addData("state: ", autoState);
                telemetry.addData("alliance: ", alliance);
                telemetry.addData("side: ", side);
                telemetry.addData("position", position);
                telemetry.addData("dt", dt);
                robot.telemetry(telemetry);
                telemetry.update();
                loop = 0;
            }
        }
    }
}

package org.firstinspires.ftc.teamcode.blucru.opmode;

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

    Gamepad currentGamepad1;
    Gamepad lastGamepad1;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        drivetrain = robot.addDrivetrain();
        intake = robot.addIntake();
        outtake = robot.addOuttake();
        purplePixelHolder = robot.addPurplePixelHolder();

        currentGamepad1 = new Gamepad();
        lastGamepad1 = new Gamepad();

        robot.init();

        while(!isStopRequested() && opModeInInit()) {
            currentGamepad1.copy(gamepad1);
            telemetry.addData("state: ", autoState);
            telemetry.addData("auto type: ", autoType);
            telemetry.addData("park: ", parkType);
            telemetry.addData("alliance: ", alliance);
            telemetry.addData("side: ", side);

            switch (autoState) {
                case INIT:
                    if(currentGamepad1.x && !lastGamepad1.x)
                        alliance = alliance == Alliance.RED ? Alliance.BLUE : Alliance.RED;

                    if(currentGamepad1.b && !lastGamepad1.b)
                        side = side == Side.CLOSE ? Side.FAR : Side.CLOSE;

                    if(currentGamepad1.y && !lastGamepad1.y) {
                        if(parkType == ParkType.CENTER)
                            parkType = ParkType.PERIMETER;
                        else if(parkType == ParkType.PERIMETER)
                            parkType = ParkType.NONE;
                        else if(parkType == ParkType.NONE)
                            parkType = ParkType.CENTER;
                    }

                    if(currentGamepad1.a && !lastGamepad1.a) {
                        if(autoType == AutoType.CENTER_CYCLE)
                            autoType = AutoType.PERIMETER_CYCLE;
                        else if(autoType == AutoType.PERIMETER_CYCLE)
                            autoType = AutoType.PRELOAD;
                        else if(autoType == AutoType.PRELOAD)
                            autoType = AutoType.PARK;
                        else if(autoType == AutoType.PARK)
                            autoType = AutoType.CENTER_CYCLE;
                    }

                    if(gamepad1.right_stick_button) {
                        autoState = AutoState.BUILD;

                        // build trajectories
                        trajectories = new Trajectories(alliance, side, autoType, parkType);

                    }

                    telemetry.addData("Press y (triangle) to cycle park position", "");
                    telemetry.addData("Press a (cross) to cycle auto type", "");
                    telemetry.addData("Press x (square) to cycle alliance", "");
                    telemetry.addData("Press b (circle) to cycle side", "");
                    telemetry.addData("Press right stick button to build trajectories", "");
                    break;
                case BUILD:
                    cvMaster = new CVMaster(hardwareMap, alliance);
                    autoState = AutoState.DETECTION;
                    break;
                case DETECTION:
                    cvMaster.detectProp();

                    position = cvMaster.propDetector.position;

                    telemetry.addData("position", cvMaster.propDetector.position);
                    telemetry.addData("average 0", cvMaster.propDetector.average0);
                    telemetry.addData("average 1", cvMaster.propDetector.average1);
                    telemetry.addData("average 2", cvMaster.propDetector.average2);
                    break;
            }

            lastGamepad1.copy(gamepad1);
            telemetry.update();
        }

        switch(alliance) {
            case RED:
                if(position == 0) {

                } else if(position == 1) {

                } else if(position == 2) {

                }
                break;
            case BLUE:
                if(position == 0) {

                } else if(position == 1) {

                } else if(position == 2) {

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
                case INIT:
                    break;
                case BUILD:
                    break;
                case DETECTION:
                    break;
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
                    break;
                case STOP:
                    Initialization.POSE = drivetrain.getPoseEstimate();
                    break;
            }

            drivetrain.updateTrajectory();
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

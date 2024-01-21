package org.firstinspires.ftc.teamcode.blucru.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blucru.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.states.AutoState;
import org.firstinspires.ftc.teamcode.blucru.states.Side;
import org.firstinspires.ftc.teamcode.blucru.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.trajectories.Trajectories;
import org.firstinspires.ftc.teamcode.blucru.vision.CVMaster;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(name = "Red Right Auto", group = "BluCru")
public class RedRightAuto extends LinearOpMode {
    Robot robot;
    Trajectories trajectories;
    CVMaster cvMaster;
    AutoState autoState;
    ElapsedTime runtime;

    int position;

    TrajectorySequence placementFar;
    TrajectorySequence placementClose;
    TrajectorySequence placementCenter;

    TrajectorySequence squareFar;
    TrajectorySequence squareCenter;
    TrajectorySequence squareClose;

    TrajectorySequence depositFar;
    TrajectorySequence depositCenter;
    TrajectorySequence depositClose;

    TrajectorySequence parkFar;
    TrajectorySequence parkCenter;
    TrajectorySequence parkClose;

    TrajectorySequence placement;
    TrajectorySequence square;
    TrajectorySequence deposit;
    TrajectorySequence park;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        trajectories = new Trajectories(Alliance.RED, Side.CLOSE);
        cvMaster = new CVMaster(hardwareMap, Alliance.RED);
//        autoState = AutoState.PLACEMENT;
        runtime = new ElapsedTime();

        robot.init();
        cvMaster.detectProp();

        telemetry.addData("robot init", "complete");
        telemetry.update();

        placementFar = trajectories.placementFar(robot);
        placementClose = trajectories.placementClose(robot);
        placementCenter = trajectories.placementCenter(robot);

        depositFar = trajectories.depositFar(robot);
        depositCenter = trajectories.depositCenter(robot);
        depositClose = trajectories.depositClose(robot);

        parkFar = trajectories.parkFar(robot);
        parkCenter = trajectories.parkCenter(robot);
        parkClose = trajectories.parkClose(robot);

        telemetry.addData("build trajectories", "complete");
        telemetry.update();

        sleep(400);


        while(!isStopRequested() && opModeInInit()) {
//            position = cvMaster.pipeline.position;

//            telemetry.addData("average0", cvMaster.pipeline.average0);
//            telemetry.addData("average1", cvMaster.pipeline.average1);
//            telemetry.addData("average2", cvMaster.pipeline.average2);
            telemetry.addData("position", position);
            telemetry.update();
        }

        switch(position) {
            case 0:
                placement = placementFar;
                square = squareFar;
                deposit = depositFar;
                park = parkFar;
                break;
            case 1:
                placement = placementCenter;
                square = squareCenter;
                deposit = depositCenter;
                park = parkCenter;
                break;
            case 2:
                placement = placementClose;
                square = squareClose;
                deposit = depositClose;
                park = parkClose;
                break;
        }

        waitForStart();

        robot.drivetrain.setPoseEstimate(trajectories.getStartPose());
//        cvMaster.stopCamera();
        runtime.reset();

        robot.drivetrain.followTrajectorySequenceAsync(placement);

        while(!isStopRequested() && opModeIsActive()) {
//            switch(autoState) {
//                case PLACEMENT:
//                    if(!robot.drivetrain.isBusy()) {
//                        robot.drivetrain.followTrajectorySequenceAsync(square);
//                        autoState = AutoState.SQUARE;
//                    }
//                    break;
//                case SQUARE:
//                    if(!robot.drivetrain.isBusy()) {
//                        robot.drivetrain.followTrajectorySequenceAsync(deposit);
//                        autoState = AutoState.DEPOSIT;
//                    }
//                    break;
//                case DEPOSIT:
//                    if(!robot.drivetrain.isBusy()) {
//                        robot.drivetrain.followTrajectorySequenceAsync(park);
//                        autoState = AutoState.PARK;
//                    }
//                    break;
//                case PARK:
//                    if(!robot.drivetrain.isBusy()) {
//                        autoState = AutoState.STOP;
//                    }
//                    break;
//                case STOP:
//                    // auto finished
//                    break;
//            }

            robot.write();
            robot.drivetrain.updateTrajectory();

            telemetry.addData("poseEstimate: ", robot.drivetrain.getPoseEstimate());
            telemetry.addData("path:", autoState);
            telemetry.addData("runtime:", runtime.seconds());
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.BluCru.opmodes;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BluCru.states.Alliance;
import org.firstinspires.ftc.teamcode.BluCru.states.Path;
import org.firstinspires.ftc.teamcode.BluCru.states.Side;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Robot;
import org.firstinspires.ftc.teamcode.BluCru.trajectories.Trajectories;
import org.firstinspires.ftc.teamcode.BluCru.vision.CVMaster;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Red Right Auto", group = "BluCru")
public class RedRightAuto extends LinearOpMode {
    Robot robot;
    Trajectories trajectories;
    CVMaster cvMaster;
    Path path;
    ElapsedTime runtime;

    int position;

    TrajectorySequence placementFar;
    TrajectorySequence placementClose;
    TrajectorySequence placementCenter;

    TrajectorySequence squareFar;
    TrajectorySequence squareCenter;
    TrajectorySequence squareClose;

    TrajectorySequence placement;
    TrajectorySequence square;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap);
        trajectories = new Trajectories(Alliance.RED, Side.CLOSE);
        cvMaster = new CVMaster(hardwareMap, Alliance.RED);
        path = Path.PLACEMENT;
        runtime = new ElapsedTime();

        robot.init();

        placementFar = trajectories.placementFar(robot);
        placementClose = trajectories.placementClose(robot);
        placementCenter = trajectories.placementCenter(robot);

        squareFar = trajectories.squareFar(robot);
        squareCenter = trajectories.squareCenter(robot);
        squareClose = trajectories.squareClose(robot);

        cvMaster.detectProp();

        while(!isStopRequested() && opModeInInit()) {
            position = cvMaster.pipeline.position;



            telemetry.addData("average0", cvMaster.pipeline.average0);
            telemetry.addData("average1", cvMaster.pipeline.average1);
            telemetry.addData("average2", cvMaster.pipeline.average2);
            telemetry.addData("position", position);
            telemetry.update();
        }

        switch(position) {
            case 0:
                placement = placementFar;
                square = squareFar;
                break;
            case 1:
                placement = placementCenter;
                square = squareCenter;
                break;
            case 2:
                placement = placementClose;
                square = squareClose;
                break;
        }

        waitForStart();

        robot.drivetrain.setPoseEstimate(trajectories.getStartPose());
        cvMaster.stopCamera();
        runtime.reset();

        robot.drivetrain.followTrajectorySequenceAsync(placement);

        while(!isStopRequested() && opModeIsActive()) {
            switch(path) {
                case PLACEMENT:
                    if(!robot.drivetrain.isBusy()) {
                        robot.drivetrain.followTrajectorySequenceAsync(square);
                        path = Path.SQUARE;
                    }
                    break;
                case SQUARE:
                    if(!robot.drivetrain.isBusy()) {
                        path = Path.PARK;
                    }
                    break;
                case PARK:
                    break;
                case STOP:
                    robot.drivetrain.setDriveSignal(new DriveSignal());
                    break;
            }

            robot.update();
            robot.drivetrain.updateTrajectory();

            telemetry.addData("poseEstimate: ", robot.drivetrain.getPoseEstimate());
            telemetry.addData("path:", path);
            telemetry.addData("runtime:", runtime.seconds());
            telemetry.update();
        }
    }
}

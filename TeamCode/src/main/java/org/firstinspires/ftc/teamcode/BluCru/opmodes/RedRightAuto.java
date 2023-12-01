package org.firstinspires.ftc.teamcode.BluCru.opmodes;

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

    double lastHeading;

    TrajectorySequence placementFar;
    TrajectorySequence placementClose;
    TrajectorySequence placementCenter;

    TrajectorySequence placement;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap);
        trajectories = new Trajectories(Alliance.RED, Side.CLOSE);
        cvMaster = new CVMaster(hardwareMap, Alliance.RED);
        path = Path.PLACEMENT;
        runtime = new ElapsedTime();

        placementFar = trajectories.placementFar(robot);
        placementClose = trajectories.placementClose(robot);
        placementCenter = trajectories.placementCenter(robot);

        cvMaster.detectProp();
        robot.init();
        robot.drivetrain.setPoseEstimate(trajectories.getStartPose());
        while(opModeInInit()) {

            telemetry.addData("average0", cvMaster.pipeline.average0);
            telemetry.addData("average1", cvMaster.pipeline.average1);
            telemetry.addData("average2", cvMaster.pipeline.average2);
            telemetry.addData("position", cvMaster.pipeline.position);
            telemetry.update();
        }

        switch(cvMaster.pipeline.position) {
            case 0:
                placement = placementFar;
                break;
            case 1:
                placement = placementCenter;
                break;
            case 2:
                placement = placementClose;
                break;
        }

        waitForStart();

        cvMaster.stopCamera();
        runtime.reset();

        while(!isStopRequested() && opModeIsActive()) {
            switch(path) {
                case PLACEMENT:
                    robot.drivetrain.followTrajectorySequenceAsync(placement);
                    path = Path.PARK;
                    break;
                case PARK:
                    break;
                case STOP:
                    robot.drivetrain.stop();
                    break;
            }

            if(robot.drivetrain.isBusy()) {
                if(robot.drivetrain.getExternalHeading() == 0 && lastHeading == 0) {
                    path = Path.STOP;
                }
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

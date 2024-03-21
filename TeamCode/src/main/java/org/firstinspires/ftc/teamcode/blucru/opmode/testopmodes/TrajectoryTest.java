package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blucru.common.states.AutoState;
import org.firstinspires.ftc.teamcode.blucru.common.states.Initialization;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.IntakeTrajectories;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Placements;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Poses;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.PreloadDeposits;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

@Autonomous(name = "trajectory test", group = "test")
public class TrajectoryTest extends LinearOpMode {
    double reflect = 1;

    Poses poses;

    IntakeTrajectories intakeTrajectories;
    Placements placements;
    PreloadDeposits preloadDeposits;
    TrajectorySequence placementFar;
    TrajectorySequence yellowDepositFar;
    TrajectorySequence intakeCenterFromFar;

    Robot robot;
    Drivetrain drivetrain;
    Outtake outtake;
    double lastTime;

    ArrayList<TrajectorySequence> trajectoryList = new ArrayList<>();
    int trajIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        drivetrain = robot.addDrivetrain(false);
        outtake = robot.addOuttake();

        poses = new Poses(reflect);
        intakeTrajectories = new IntakeTrajectories(reflect);
        preloadDeposits = new PreloadDeposits(reflect);
        placements = new Placements(reflect);

        placementFar = placements.placementBackdropFar(robot);
        yellowDepositFar = preloadDeposits.depositFromBackdropFar(robot);
        intakeCenterFromFar = intakeTrajectories.intakeCenterFromFar(robot, 3);

        trajectoryList.add(placementFar);
        trajectoryList.add(yellowDepositFar);
        trajectoryList.add(intakeCenterFromFar);

        robot.init();


        waitForStart();

        int state = 0;

        drivetrain.setPoseEstimate(Poses.BACKDROP_STARTING_POSE);

        while(opModeIsActive()) {
            robot.read();

            switch(state) {
                case 0:
                    if(!drivetrain.isBusy()) {
                        if(trajIndex >= trajectoryList.size()) {
                            state = 1;
                            break;
                        } else {
                            drivetrain.followTrajectorySequenceAsync(trajectoryList.get(trajIndex));
                            trajIndex++;
                        }
                    }
                    break;
                case 1:
                    break;
            }

            robot.write();
            drivetrain.updateTrajectory();

            double dt = System.currentTimeMillis() - lastTime;
            lastTime = System.currentTimeMillis();
            telemetry.addData("# of trajectories: ", trajectoryList.size());
            telemetry.addData("index", trajIndex);
            telemetry.addData("loop time", dt);
            robot.telemetry(telemetry);
            telemetry.update();
        }
    }
}
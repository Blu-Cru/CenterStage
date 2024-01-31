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
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

@Autonomous(name = "trajectory test", group = "test")
public class TrajectoryTest extends LinearOpMode {
    public static double reflect = -1;

    private static double stackSetupX = -50;
    private static double stackX = -58;
    private static double depositX = 52;
    private static double depositSetupX = 45;

    private static Pose2d closeStartingPose = new Pose2d(12, -62 * reflect, Math.toRadians(-90 * reflect));
    private static Pose2d closePlacementFarPose = new Pose2d(5, -39 * reflect, Math.toRadians(-45 * reflect));
    private static Pose2d closePlacementClosePose = new Pose2d(17.5, -40 * reflect, Math.toRadians(-135 * reflect));
    private static Pose2d closePlacementCenterPose = new Pose2d(15, -31 * reflect, Math.toRadians(-90 * reflect));

    private static Pose2d farStartingPose = new Pose2d(-36, -62 * reflect, Math.toRadians(-90 * reflect));
    private static Pose2d farPlacementFarPose = new Pose2d(-56, -36 * reflect, Math.toRadians(180));
    private static Pose2d farPlacementClosePose = new Pose2d(-31, -40 * reflect, Math.toRadians(-135 * reflect));
    private static Pose2d farPlacementCenterPose = new Pose2d(-33, -31 * reflect, Math.toRadians(-90 * reflect));

    private static Pose2d squareFarPose = new Pose2d(50, -29 * reflect, Math.toRadians(180));
    private static Pose2d squareCenterPose = new Pose2d(50, -36 * reflect, Math.toRadians(180));
    private static Pose2d squareClosePose = new Pose2d(50, -43 * reflect, Math.toRadians(180));

    private static Pose2d depositFarPose = new Pose2d(depositX, -29 * reflect, Math.toRadians(180));
    private static Pose2d depositCenterPose = new Pose2d(depositX, -36 * reflect, Math.toRadians(180));
    private static Pose2d depositClosePose = new Pose2d(depositX, -43 * reflect, Math.toRadians(180));

    private static Pose2d alignClosePose = new Pose2d(-58, -36*reflect, Math.toRadians(180));

    private static Pose2d parkPose = new Pose2d(60, -12 * reflect, Math.toRadians(180));

    private static TrajectoryVelocityConstraint slowVelocity = SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(180), DriveConstants.TRACK_WIDTH);
    private static TrajectoryVelocityConstraint normalVelocity = SampleMecanumDrive.getVelocityConstraint(28, Math.toRadians(180), DriveConstants.TRACK_WIDTH);
    private static TrajectoryVelocityConstraint fastVelocity = SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(200), DriveConstants.TRACK_WIDTH);

    TrajectorySequence closeCloseAuto;
    TrajectorySequence centerOfTile;

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

        robot.init();

        centerOfTile = drivetrain.trajectorySequenceBuilder(closeStartingPose)
                .lineTo(new Vector2d(12, -36))
                .build();

        closeCloseAuto = getCloseCloseAuto();

        TrajectorySequence release = drivetrain.trajectorySequenceBuilder(closeStartingPose)
                // lift
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.YELLOW_POS);
                })
                // wrist back
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.outtake.extendWrist();
                })
                // turn turret
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    robot.outtake.setTurretAngle(210);
                })
                .waitSeconds(0.8)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.unlock();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CLEAR_POS);
                    robot.outtake.setTurretAngle(270);
                })
                .waitSeconds(0.8)
                .build();

        TrajectorySequence retract = drivetrain.trajectorySequenceBuilder(closeStartingPose)
                // retract wrist
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.retractWrist();
                })
                // retract wrist
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(0);
                })
                .waitSeconds(1.5)
                .build();

        trajectoryList.add(release);
        trajectoryList.add(retract);

        waitForStart();

        int state = 0;

        drivetrain.setPoseEstimate(closeStartingPose);

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

    public TrajectorySequence getCloseCloseAuto() {
        return drivetrain.trajectorySequenceBuilder(closeStartingPose)
                // drive to placement
                .setVelConstraint(fastVelocity)
                .setTangent(Math.toRadians(90 * reflect))
                .lineTo(new Vector2d(24, -40 * reflect))
                // placement

                .waitSeconds(0.5)
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(30, -44 * reflect), 0)
                .splineToSplineHeading(new Pose2d(depositSetupX, -43 * reflect, Math.toRadians(180)), 0)
                // lift

                .splineToConstantHeading(new Vector2d(depositX, -43 * reflect), 0)
                // deposit and retract

                .waitSeconds(1.2)

                // drive to far stack
                .setVelConstraint(fastVelocity)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(25, -10 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(stackSetupX, -10 * reflect), Math.toRadians(180))
                // start intaking

                .setVelConstraint(slowVelocity)
                .splineToConstantHeading(new Vector2d(stackX, -10 * reflect), Math.toRadians(180))
                .waitSeconds(1.5)
                // lock and retract

                // drive to deposit
                .setTangent(0)
                .setVelConstraint(fastVelocity)
                .splineToConstantHeading(new Vector2d(20, -10 * reflect), 0)
                .splineToConstantHeading(new Vector2d(depositSetupX, -29 * reflect), 0)
                // lift

                .splineToConstantHeading(new Vector2d(depositX, -29 * reflect), 0)
                // deposit and retract

                .waitSeconds(1.2)

                // drive to far stack
                .setVelConstraint(fastVelocity)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(25, -10 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(stackSetupX, -10 * reflect), Math.toRadians(180))
                // start intaking

                .setVelConstraint(slowVelocity)
                .splineToConstantHeading(new Vector2d(stackX, -10 * reflect), Math.toRadians(180))
                .waitSeconds(1.5)
                // lock and retract

                // drive to deposit
                .setTangent(0)
                .setVelConstraint(fastVelocity)
                .splineToConstantHeading(new Vector2d(20, -10 * reflect), 0)
                .splineToConstantHeading(new Vector2d(depositSetupX, -29 * reflect), 0)
                // lift

                .splineToConstantHeading(new Vector2d(depositX, -29 * reflect), 0)
                // deposit and retract

                .waitSeconds(1.2)
                .build();
    }
}

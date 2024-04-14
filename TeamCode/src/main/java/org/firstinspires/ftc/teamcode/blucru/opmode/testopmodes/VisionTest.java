package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.blucru.common.vision.CVMaster;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="vision test", group="test")
public class VisionTest extends LinearOpMode {
    CVMaster cvMaster;
    Alliance alliance = Alliance.BLUE;

    String status;

    boolean lastLB1 = false;

    @Override
    public void runOpMode() {
        cvMaster = new CVMaster(hardwareMap, Alliance.BLUE);

        telemetry.addLine("camera starting");
        telemetry.update();

        status = "stopped";

        cvMaster.detectProp();
        // Init
        while (opModeInInit() && !isStopRequested()) {

            if(gamepad1.b) {
                status = "detecting prop";
                cvMaster.detectProp();
                FtcDashboard.getInstance().startCameraStream((CameraStreamSource) cvMaster.visionPortal, 30);
            }
            if(gamepad1.x) {
                status = "detecting tag";
                cvMaster.detectTag();
                FtcDashboard.getInstance().startCameraStream((CameraStreamSource) cvMaster.visionPortal, 30);
            }
            if(gamepad1.a) {
                status = "stopped";
                cvMaster.stop();
                FtcDashboard.getInstance().stopCameraStream();
            }

            switch (status) {
                case "detecting prop":
                    telemetry.addData("alliance", alliance);
                    telemetry.addData("Average0", cvMaster.propDetector.average0);
                    telemetry.addData("Average1", cvMaster.propDetector.average1);
                    telemetry.addData("Average2", cvMaster.propDetector.average2);
                    telemetry.addData("position", cvMaster.propDetector.position);
                    break;
                case "detecting tag":
                    List<AprilTagDetection> currentDetections = cvMaster.tagDetector.getDetections();

                    AprilTagDetection closestDetection;
                    double closestDistance = Double.MAX_VALUE;
                    if(currentDetections.size() > 0) {
                        closestDetection = currentDetections.get(0);
                        closestDistance = Math.hypot(closestDetection.ftcPose.x, closestDetection.ftcPose.y);

                        for (AprilTagDetection detection : currentDetections) {
                            if(detection.id == 3) {
                                Pose2d pose3 = AprilTagLocalizer.getRobotPose(detection);
                                telemetry.addLine(String.format("found tag 3, pose estimate (x, y, heading): ", pose3.getX(), pose3.getY(), pose3.getHeading()));
                            }

                            if(detection.id == 4) {
                                Vector2d btt4 = AprilTagLocalizer.getRobotToTagVector(detection.ftcPose.x, detection.ftcPose.y);
                                Pose2d pose4 = AprilTagLocalizer.getRobotPose(4, detection.ftcPose.x, detection.ftcPose.y, Math.toRadians(detection.ftcPose.yaw));
                                telemetry.addLine(String.format("found tag 4"));
                                telemetry.addLine("robot to tag: ");
                                telemetry.addData("x:", btt4.getX());
                                telemetry.addData("y:", btt4.getY());

                                telemetry.addLine("pose estimate:");
                                telemetry.addData("x:", pose4.getX());
                                telemetry.addData("y:", pose4.getY());
                                telemetry.addData("headnig", Math.toDegrees(pose4.getHeading()));
                            }

                            if (detection.metadata != null) {
                                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                            } else {
                                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                            }
                        }   // end for() loop
                        break;
                    }
            }

            telemetry.addData("Status", status);
            telemetry.update();
        }
        waitForStart();

        cvMaster.stop();

        // Run
        while (opModeIsActive()) {

        }
    }
}
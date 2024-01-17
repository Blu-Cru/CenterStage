package org.firstinspires.ftc.teamcode.blucru.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blucru.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.vision.CVMaster;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class VisionPortalTest extends LinearOpMode {
    CVMaster cvMaster;

    @Override
    public void runOpMode() throws InterruptedException {
        cvMaster = new CVMaster(hardwareMap, Alliance.RED);
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.b) {
                cvMaster.detectTag();
            } else if(gamepad1.x) {
                cvMaster.detectProp();
            } else if(gamepad1.a) {
                cvMaster.stop();
            }

            List<AprilTagDetection> detections = cvMaster.tagDetector.getDetections();
            telemetry.addData("Detections", detections.size());
            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null) {
                    telemetry.addData("Detection", detection.id + " " + detection.metadata.name);
                    telemetry.addData("XYZ (inch)", detection.ftcPose.x + " " + detection.ftcPose.y + " " + detection.ftcPose.z);
                    telemetry.addData("PRY (deg)", detection.ftcPose.pitch + " " + detection.ftcPose.roll + " " + detection.ftcPose.yaw);
                    telemetry.addData("RBE (inch, deg, deg)", detection.ftcPose.range + " " + detection.ftcPose.bearing + " " + detection.ftcPose.elevation);
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }
            telemetry.addData("average0", cvMaster.propDetector.average0);
            telemetry.addData("average1", cvMaster.propDetector.average1);
            telemetry.addData("average2", cvMaster.propDetector.average2);
            telemetry.addData("position", cvMaster.propDetector.position);
            telemetry.update();
        }
    }
}

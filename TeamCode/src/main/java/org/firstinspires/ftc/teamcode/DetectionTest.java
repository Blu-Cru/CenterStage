package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="DetectionTest", group="Linear Opmode")
public class DetectionTest extends LinearOpMode {
    CVMaster cvMaster;

        @Override
        public void runOpMode() {
            cvMaster = new CVMaster(hardwareMap);
            cvMaster.detectProp();
            // Init
            while (opModeInInit()) {
                telemetry.addData("Status", "Initialized");
                telemetry.addData("Average0", BluePropDetectionPipeline.average0);
                telemetry.addData("Average1", BluePropDetectionPipeline.average1);
                telemetry.addData("Average2", BluePropDetectionPipeline.average2);
                telemetry.update();
            }
            waitForStart();
            // Run
            while (opModeIsActive()) {
                telemetry.addData("Status", "Running");
                telemetry.update();
            }
        }
}

package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.BluCru.Alliance;
import org.firstinspires.ftc.teamcode.BluCru.vision.CVMaster;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="DetectionTest", group="Linear Opmode")
public class DetectionTest extends LinearOpMode {
    CVMaster cvMaster;
    Alliance alliance = Alliance.BLUE;

    Gamepad lastGamepad1 = new Gamepad();
    Gamepad lastGamepad2 = new Gamepad();

    @Override
    public void runOpMode() {
        cvMaster = new CVMaster(hardwareMap);
        cvMaster.detectProp();
        // Init
        while (opModeInInit()) {
            if(gamepad1.left_bumper && !lastGamepad1.left_bumper) {
                if(alliance == Alliance.BLUE) {
                    alliance = Alliance.RED;
                } else {
                    alliance = Alliance.BLUE;
                }
                cvMaster.setAlliance(alliance);
            }

            telemetry.addData("Status", "Initialized");
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Average0", cvMaster.pipeline.average0);
            telemetry.addData("Average1", cvMaster.pipeline.average1);
            telemetry.addData("Average2", cvMaster.pipeline.average2);
            telemetry.update();

            lastGamepad1.copy(gamepad1);
            lastGamepad2.copy(gamepad2);
        }
        waitForStart();
        // Run
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
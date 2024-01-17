package org.firstinspires.ftc.teamcode.blucru.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.blucru.states.Alliance;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

public class CVMaster {
    public static double fx = 1059.73;
    public static double fy = 1059.73;
    public static double cx = 625.979;
    public static double cy = 362.585;

    ExposureControl exposureControl;
    GainControl gainControl;

    public VisionPortal visionPortal;
    public VisionProcessor propDetector;
    public AprilTagProcessor tagDetector;


    public CVMaster(HardwareMap hardwareMap, Alliance alliance) {
        this.propDetector = new PropDetectionProcessor(alliance);

        this.tagDetector = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(fx, fy, cx, cy)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(propDetector)
                .addProcessor(tagDetector)
                .build();
        visionPortal.setProcessorEnabled(propDetector, false);
        visionPortal.setProcessorEnabled(tagDetector, false);
        visionPortal.stopStreaming();

        exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//        exposureControl.setMode(ExposureControl.Mode.Manual);
//        exposureControl.setExposure(50, TimeUnit.MILLISECONDS);

        gainControl = visionPortal.getCameraControl(GainControl.class);
//        gainControl.setGain(3200);
    }

    public void init() {

    }

    public void detectProp() {
        visionPortal.resumeStreaming();
        visionPortal.setProcessorEnabled(propDetector, true);
        visionPortal.setProcessorEnabled(tagDetector, false);
        exposureControl.setMode(ExposureControl.Mode.ContinuousAuto);
    }
}
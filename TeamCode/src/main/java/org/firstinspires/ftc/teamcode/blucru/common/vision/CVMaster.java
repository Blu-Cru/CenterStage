package org.firstinspires.ftc.teamcode.blucru.common.vision;

import static java.lang.Thread.sleep;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Subsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@Config
public class CVMaster implements Subsystem {
    public static double fx = 1059.73;
    public static double fy = 1059.73;
    public static double cx = 625.979;
    public static double cy = 362.585;
    public static int GAIN = 10;
    public static long EXPOSURE = 4; // ms
    public static double FOCUS = 1;

    ExposureControl exposureControl;
    GainControl gainControl;
    FocusControl focusControl;

    public VisionPortal visionPortal;
    public PropDetectionProcessor propDetector;
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
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(propDetector)
                .addProcessor(tagDetector)
                .build();
        visionPortal.setProcessorEnabled(propDetector, false);
        visionPortal.setProcessorEnabled(tagDetector, false);

        exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        gainControl = visionPortal.getCameraControl(GainControl.class);
        focusControl = visionPortal.getCameraControl(FocusControl.class);
    }

    public void init() {

    }

    public void read() {

    }

    public void write() {

    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("camera state", visionPortal.getCameraState());
        telemetry.addData("Prop detector enabled", visionPortal.getProcessorEnabled(propDetector));
        if(visionPortal.getProcessorEnabled(propDetector)) {
            telemetry.addData("POSITION", propDetector.position);
        }
        telemetry.addData("Tag detector enabled", visionPortal.getProcessorEnabled(tagDetector));
        if(visionPortal.getProcessorEnabled(tagDetector)) {
            telemetry.addData("Detection processing time", tagDetector.getPerTagAvgPoseSolveTime());
        }
    }

    public void detectProp() {
        visionPortal.resumeStreaming();

        visionPortal.setProcessorEnabled(propDetector, true);
        visionPortal.setProcessorEnabled(tagDetector, false);
        exposureControl.setMode(ExposureControl.Mode.ContinuousAuto);
        focusControl.setMode(FocusControl.Mode.ContinuousAuto);
    }

    public void detectTag() {
        visionPortal.resumeStreaming();

        visionPortal.setProcessorEnabled(propDetector, false);
        visionPortal.setProcessorEnabled(tagDetector, true);

        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(EXPOSURE, TimeUnit.MILLISECONDS);
        gainControl.setGain(GAIN);
        focusControl.setFocusLength(FOCUS);
    }

    public void stop() {
        visionPortal.setProcessorEnabled(propDetector, false);
        visionPortal.setProcessorEnabled(tagDetector, false);
        visionPortal.stopStreaming();
    }

    public void setCameraExposure(double exposure) {
        exposureControl.setExposure((long) exposure, TimeUnit.MILLISECONDS);
    }

    public void setCameraGain(double gain) {
        gainControl.setGain((int) gain);
    }

    public void setCameraFocus(double focus) {
        focusControl.setFocusLength(focus);
    }
}
package org.firstinspires.ftc.teamcode.BluCru.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

public class PropDetectionProcessor implements VisionProcessor {
    public void init() {

    }

    public void init(int width, int height, CameraCalibration cameraCalibration) {

    }

    public Mat processFrame(Mat mat, long captureTime) {
        return mat;
    }

    public void onDrawFrame(Canvas canvas,
                            int width,
                            int height,
                            float bmpToCanvasPx,
                            float bmpDensity,
                            Object userContext) {

    }
}

package org.firstinspires.ftc.teamcode.blucru.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.blucru.states.Alliance;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class CVMaster {
    private Alliance alliance = Alliance.BLUE;
    private OpenCvWebcam webcam;
    public PropDetectionPipeline pipeline;
    private HardwareMap hardwareMap;


    public CVMaster(HardwareMap hardwareMap, Alliance alliance) {
        this.hardwareMap = hardwareMap;
        this.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"));
        this.alliance = alliance;
    }

    public void detectProp() {
        pipeline = new PropDetectionPipeline(1280, 720, alliance);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public void stopCamera() {
        webcam.stopStreaming();
    }

    public boolean webcamIsCalibrated() {
        return !webcam.equals(null);
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }
}
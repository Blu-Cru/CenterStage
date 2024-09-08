package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.vision.atag.AprilTagPoseGetter;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(name="April Tag Test", group="test")
public class AprilTagTest extends BluLinearOpMode {
    private enum State {
        IDLE,
        DETECTING
    }
    public static double CAMERA_EXPOSURE = 50;
    public static double CAMERA_GAIN = 10;
    public static double CAMERA_FOCUS = 1;

    boolean lastA = false;
    boolean lastX = false;

    boolean exposure, focus, gain;

    State state = State.IDLE;

    @Override
    public void initialize() {
        addDrivetrain(true);
        addCVMaster();
        enableFTCDashboard();

        exposure = false;
        focus = false;
        gain = false;
    }

    public void initLoop() {
        exposure = cvMaster.setCameraExposure(CAMERA_EXPOSURE);
        gain = cvMaster.setCameraGain(CAMERA_GAIN);
        focus = cvMaster.setCameraFocus(CAMERA_FOCUS);

        if (gamepad1.a && !lastA) {
            cvMaster.detectTag();
            state = State.DETECTING;
            FtcDashboard.getInstance().startCameraStream((CameraStreamSource) cvMaster.visionPortal, 30);
        }
        lastA = gamepad1.a;

        if (gamepad1.x && !lastX) {
            cvMaster.stop();
            state = State.IDLE;
            FtcDashboard.getInstance().stopCameraStream();
        }
        lastX = gamepad1.x;

        try {
            drivetrain.setPoseEstimate(AprilTagPoseGetter.getRobotPoseAtTimeOfFrame(cvMaster.tagDetector.getDetections()));
        } catch (Exception e) {

        }
        drivetrain.ftcDashDrawCurrentPose();
    }

    public void telemetry() {
        telemetry.addData("state:", state);
        telemetry.addData("exposure", exposure);
        telemetry.addData("gain", gain);
        telemetry.addData("focus", focus);
    }
}

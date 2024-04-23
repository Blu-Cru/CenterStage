package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.blucru.common.util.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

@Config
@TeleOp(name="April Tag Test", group="test")
public class AprilTagTest extends BCLinearOpMode {
    enum State {
        IDLE,
        DETECTING
    }
    public static double CAMERA_EXPOSURE = 50;
    public static double CAMERA_GAIN = 10;
    public static double CAMERA_FOCUS = 1;

    boolean lastA = false;
    boolean lastX = false;

    State state = State.IDLE;

    @Override
    public void initialize() {
        addDrivetrain(true);
        addCVMaster();
        enableFTCDashboard();
    }

    public void initLoop() {
        cvMaster.setCameraExposure(CAMERA_EXPOSURE);
        cvMaster.setCameraGain(CAMERA_GAIN);
        cvMaster.setCameraFocus(CAMERA_FOCUS);

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

        drivetrain.setPoseEstimate(AprilTagLocalizer.getRobotPose(cvMaster.tagDetector.getDetections()));
        drivetrain.ftcDashDrawPose();
    }

    public void telemetry() {
        telemetry.addData("state:", state);
    }
}

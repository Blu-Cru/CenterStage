package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.vision.CVMaster;

@Autonomous(name="vision test", group="Linear Opmode")
public class VisionTest extends LinearOpMode {
    CVMaster cvMaster;
    Alliance alliance = Alliance.BLUE;

    String status;

    Gamepad lastGamepad1 = new Gamepad();
    Gamepad lastGamepad2 = new Gamepad();

    @Override
    public void runOpMode() {
        cvMaster = new CVMaster(hardwareMap, Alliance.BLUE);

        telemetry.addLine("camera starting");
        telemetry.update();

        sleep(1000);

        cvMaster.detectProp();
        // Init
        while (opModeInInit() && !isStopRequested()) {
            if(gamepad1.left_bumper && !lastGamepad1.left_bumper) {
                if(alliance == Alliance.BLUE) {
                    alliance = Alliance.RED;
                } else {
                    alliance = Alliance.BLUE;
                }
                cvMaster.propDetector.setAlliance(alliance);
            }

            telemetry.addLine("left bumper to change alliance");
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Average0", cvMaster.propDetector.average0);
            telemetry.addData("Average1", cvMaster.propDetector.average1);
            telemetry.addData("Average2", cvMaster.propDetector.average2);
            telemetry.update();

            lastGamepad1.copy(gamepad1);
            lastGamepad2.copy(gamepad2);
        }
        waitForStart();

        cvMaster.stop();

        // Run
        while (opModeIsActive()) {

            if(gamepad1.b) {
                status = "detecting prop";
                cvMaster.detectProp();
            }
            if(gamepad1.x) {
                status = "detecting tag";
                cvMaster.detectTag();
            }
            if(gamepad1.a) {
                status = "stopped";
                cvMaster.stop();
            }

            telemetry.addData("Status", status);
            telemetry.update();
        }
    }
}
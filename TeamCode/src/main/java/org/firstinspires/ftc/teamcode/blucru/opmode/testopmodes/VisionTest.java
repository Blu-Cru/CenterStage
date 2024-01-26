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

    boolean lastLB1 = false;

    @Override
    public void runOpMode() {
        cvMaster = new CVMaster(hardwareMap, Alliance.BLUE);

        telemetry.addLine("camera starting");
        telemetry.update();


        cvMaster.detectProp();
        // Init
        while (opModeInInit() && !isStopRequested()) {
            if(gamepad1.left_bumper && !lastLB1) {
                if(alliance == Alliance.BLUE) {
                    alliance = Alliance.RED;
                } else {
                    alliance = Alliance.BLUE;
                }
                cvMaster.propDetector.setAlliance(alliance);
            }
            lastLB1 = gamepad1.left_bumper;

            telemetry.addLine("left bumper to change alliance");
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Average0", cvMaster.propDetector.average0);
            telemetry.addData("Average1", cvMaster.propDetector.average1);
            telemetry.addData("Average2", cvMaster.propDetector.average2);
            telemetry.addData("position", cvMaster.propDetector.position);
            telemetry.update();
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
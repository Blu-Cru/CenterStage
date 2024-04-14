package org.firstinspires.ftc.teamcode.blucru.common.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.states.Initialization;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.DrivetrainMigrated;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.IntakeColorSensors;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Lock;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Plane;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.PurplePixelHolder;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Turret;
import org.firstinspires.ftc.teamcode.blucru.common.vision.CVMaster;

public abstract class BCLinearOpMode extends LinearOpMode {
    public Alliance alliance;
    public Robot robot;
    public DrivetrainMigrated drivetrain;
    public Outtake outtake;
    public Lift lift;
    public Turret turret;
    public Intake intake;
    public IntakeWrist intakeWrist;
    public IntakeColorSensors intakeColorSensors;
    public Plane plane;
    public Hanger hanger;
    public PurplePixelHolder purplePixelHolder;
    public Lock lock;
    public CVMaster cvMaster;

    ElapsedTime runtime;
    double lastTime;
    double loopTimeSum;
    int loopTimeCount;
    double lastTelemetryTime;

    public final void runOpMode() throws InterruptedException {
        alliance = Initialization.ALLIANCE;
        robot = Robot.getInstance();
        robot.create(hardwareMap);
        initialize();
        robot.init();
        while(opModeInInit()) {
            initLoop();
            telemetry();
            telemetry.update();
        }
        waitForStart();
        onStart();
        runtime = new ElapsedTime(); // start timer
        while (!isStopRequested() && opModeIsActive()) {
            robot.read();
            periodic();
            robot.write();

            // calculate average loop time
            loopTimeSum += runtime.milliseconds() - lastTime;
            lastTime = runtime.milliseconds();
            loopTimeCount++;

            if(timeSince(lastTelemetryTime) > 100) { // update telemetry every 0.1 seconds
                lastTelemetryTime = runtime.milliseconds();
                telemetry();
                robot.telemetry(telemetry);

                telemetry.addData("alliance:", alliance);
                double loopTimeAvg = loopTimeSum / loopTimeCount;
                resetLoopTime();
                telemetry.addData("Loop Time", loopTimeAvg);
                telemetry.update();
            }
        }

        end();
        Robot.kill();
    }

    // methods to be overriden
    public void initialize() {}
    public void initLoop() {}
    public void onStart() {}
    public void periodic() {}
    public void telemetry() {}
    public void end() {}

    public void addDrivetrain(boolean isTeleOp) {drivetrain = robot.addDrivetrain(isTeleOp);}

    public void addOuttake() {outtake = robot.addOuttake();}
    public void addLocks() {lock = robot.addLocks();}

    public void addIntake() {intake = robot.addIntake();}

    public void addIntakeWrist() {intakeWrist = robot.addIntakeWrist();}

    public void addIntakeColorSensors() {intakeColorSensors = robot.addIntakeColorSensors();}

    public void addPlane() {plane = robot.addPlane();}

    public void addHanger() {hanger = robot.addHanger();}

    public void addPurplePixelHolder() {purplePixelHolder = robot.addPurplePixelHolder();}

    public void addCVMaster() {cvMaster = robot.addCVMaster(Initialization.ALLIANCE);}

    public void enableFTCDashboard() {telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);}

    public double currentSecs() {return runtime.seconds();}

    public double currentTime() {return runtime.milliseconds();}

    public double secsSince(double timeSecs) {
        return runtime.seconds() - timeSecs;
    }

    public double timeSince(double timeMillis) {
        return runtime.milliseconds() - timeMillis;
    }

    private void resetLoopTime() {
        loopTimeSum = 0;
        loopTimeCount = 0;
    }
}

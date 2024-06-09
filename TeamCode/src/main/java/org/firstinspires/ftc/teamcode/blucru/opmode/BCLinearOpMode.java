package org.firstinspires.ftc.teamcode.blucru.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.IntakeColorSensors;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang.Hanger;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.IntakeWrist;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.outtake.Lift;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.outtake.Lock;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.plane.Plane;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.purple.PurplePixelHolder;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.outtake.Turret;
import org.firstinspires.ftc.teamcode.blucru.common.util.wrappers.StickyGamepad;
import org.firstinspires.ftc.teamcode.blucru.common.vision.CVMaster;

public abstract class BCLinearOpMode extends LinearOpMode {
    public Alliance alliance;
    public Robot robot;
    public Drivetrain drivetrain;
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

    public StickyGamepad stickyG1;
    public StickyGamepad stickyG2;

    ElapsedTime runtime;
    double lastTime;
    double loopTimeSum;
    int loopTimeCount;
    double lastTelemetryTime;

    boolean telemetryOptimized = false;

    public final void runOpMode() throws InterruptedException {
        alliance = Globals.alliance;
        stickyG1 = new StickyGamepad(gamepad1);
        stickyG2 = new StickyGamepad(gamepad2);
        robot = Robot.getInstance();
        robot.create(hardwareMap);
        Globals.setVoltage(robot.getVoltage());
        initialize();
        robot.init();
        while(opModeInInit()) {
            stickyG1.update();
            stickyG2.update();
            initLoop();
            try {
                telemetry();
                telemetry.update();
            } catch (Exception e) {}
        }
        waitForStart();
        onStart();
        runtime = new ElapsedTime(); // start timer
        while (!isStopRequested() && opModeIsActive()) {
            stickyG1.update();
            stickyG2.update();
            robot.read();
            periodic();
            CommandScheduler.getInstance().run();
            robot.write();

            // calculate average loop time
            loopTimeSum += runtime.milliseconds() - lastTime;
            lastTime = runtime.milliseconds();
            loopTimeCount++;

            if(telemetryOptimized) {
                if (timeSince(lastTelemetryTime) > 100) { // update telemetry every 0.1 seconds
                    lastTelemetryTime = runtime.milliseconds();
                    telemetry();
                    robot.telemetry(telemetry);

                    telemetry.addData("alliance:", alliance);
                    double loopTimeAvg = loopTimeSum / loopTimeCount;
                    resetLoopTime();
                    telemetry.addData("Loop Time", loopTimeAvg);
                    telemetry.update();
                }
            } else {
                telemetry();
                robot.telemetry(telemetry);
                telemetry.addData("alliance:", alliance);
                telemetry.addData("Loop Time", loopTimeSum / loopTimeCount);
                resetLoopTime();
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

    public void addLift() {lift = robot.addLift();}

    public void addLocks() {lock = robot.addLocks();}

    public void addIntake() {intake = robot.addIntake();}

    public void addIntakeWrist() {intakeWrist = robot.addIntakeWrist();}

    public void addIntakeColorSensors() {intakeColorSensors = robot.addIntakeColorSensors();}

    public void addPlane() {plane = robot.addPlane();}

    public void addHanger() {hanger = robot.addHanger();}

    public void addPurplePixelHolder() {purplePixelHolder = robot.addPurplePixelHolder();}

    public void addCVMaster() {cvMaster = robot.addCVMaster(Globals.alliance);}

    public void enableFTCDashboard() {telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);}

    public void optimizeTelemetry() {telemetryOptimized = true;}

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

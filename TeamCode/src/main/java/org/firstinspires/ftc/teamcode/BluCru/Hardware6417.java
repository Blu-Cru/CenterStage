package org.firstinspires.ftc.teamcode.BluCru;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BluCru.Constants;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Vector;

public class Hardware6417 {
    public DcMotorEx slider, auxSlider;
    public ServoControllerEx wristController;
    public Servo wrist;
    public CRServo wheels;
    public MecanumDrive drive;
    public IMU imu;
    public HardwareMap hwMap;
//  heading offset is the heading set to forward on the field
    private double headingOffset;
    private PIDController slidePID;

    public Hardware6417(HardwareMap hardwareMap) {
        hwMap = hardwareMap;
    }

    public void initSlides() {
// init PID
        slidePID = new PIDController(Constants.sliderP, Constants.sliderI, Constants.sliderD);

        slider  = hwMap.get(DcMotorEx.class, "slider");
        auxSlider = hwMap.get(DcMotorEx.class, "auxSlider");

//      set directions
        slider.setDirection(DcMotorSimple.Direction.FORWARD);
        auxSlider.setDirection(DcMotorSimple.Direction.REVERSE);

        //set all motors to zero power
        slider.setPower(0);
        auxSlider.setPower(0);

        slider.setTargetPosition(0);
        auxSlider.setTargetPosition(0);

        //set brake behavior
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        auxSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        auxSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initIMU() {
        imu = hwMap.get(IMU.class, "imu");
    }

    public void initIntake() {
        wrist       = hwMap.get(Servo.class, "wrist");
        wristController = (ServoControllerEx) wrist.getController();

        wheels     = hwMap.get(CRServo.class, "wheels");
    }

    public void initDrive(Pose2d pose) {
        drive = new MecanumDrive(hwMap, pose);
    }

    public void autoWrist(double position) {
//      enable servo pwm if it is not already enabled
        if(wristController.getPwmStatus() != ServoControllerEx.PwmStatus.ENABLED) {
            wristController.setServoPwmEnable(wrist.getPortNumber());
        }
        if(wrist.getPosition() != position) {
            wrist.setPosition(position);
        }
    }

    public void setWheelPowers(double power) {
        wheels.setPower(power);
    }

    public void stopWrist() {
//      disable servo pwm if it is not already disabled
        if(wristController.getPwmStatus() != ServoControllerEx.PwmStatus.DISABLED) {
            wristController.setServoPwmDisable(wrist.getPortNumber());
        }
    }

    public boolean slideOuttakeReady() {
        return Math.abs(slider.getTargetPosition() - slider.getCurrentPosition()) < Constants.sliderOuttakeDelta;
    }

    public void autoSlider(int position) {
        int target = Range.clip(position, Constants.sliderMinPos, Constants.sliderMaxPos);
        double power = slidePID.calculate(slider.getCurrentPosition(), target);
        setSlidePowers(power);
    }

    public void setSlidePowers(double power) {
        if(slider.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
            slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(auxSlider.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            auxSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        slider.setPower(power);
        auxSlider.setPower(power);
    }

    public void resetSliders() {
        slider.setPower(0);
        auxSlider.setPower(0);

        slider.setTargetPosition(0);
        auxSlider.setTargetPosition(0);

        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        auxSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        auxSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean sliderIntakeReady() {
        return slider.getCurrentPosition() < Constants.sliderIntakeDelta;
    }

    public void holonomicDrive(double horz, double vert, double rotate, double driveSpeed, double heading) {
        Vector2d input = new Vector2d(horz, vert);
        input = rotateVector(input, -Math.toRadians(90));

        if(Math.max(Math.max(Math.abs(vert), Math.abs(horz)), Math.abs(rotate)) > 0.1) {
            drive.setDrivePowers(new PoseVelocity2d(input, rotate));
        } else {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        }
    }

    /*public void holonomicDrive(double vert, double horz, double rotate, double driveSpeed, double heading) {
        Vector2d input = new Vector2d(horz, vert).rotated(-heading - Math.toRadians(90));
        setWeightedDrivePower(new Pose2d(input.getX() * driveSpeed, input.getY() * driveSpeed, rotate * driveSpeed));
    }*/
    public Vector2d rotateVector(Vector2d vector, double angle) {
        return new Vector2d(vector.component1() * Math.cos(angle) - vector.component2() * Math.sin(angle), vector.component1() * Math.sin(angle) + vector.component2() * Math.cos(angle));
    }

    public void telemetry(Telemetry tele) {
        tele.addData("slider position: ", slider.getCurrentPosition());
        tele.addData("slider target pos: ", slider.getTargetPosition());
        tele.addData("aux slider target pos: ", auxSlider.getTargetPosition());
        tele.addData("wrist position: ", wrist.getPosition());
        tele.addData("wheels power: ", wheels.getPower());
    }
}
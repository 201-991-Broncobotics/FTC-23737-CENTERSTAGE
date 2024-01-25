package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutonHardware {
    public final HardwareMap map;

    public final Telemetry telemetry;
    public final BNO055IMU imu;


    public final DcMotorEx RF, RB, LF, LB, LA, RA;


    public final CRServo RFServo, RBServo, LFServo, LBServo;


    // VFB Position PID variables
    double PosIntegralSum = 0;
    double PosKp = 0;
    double PosKi = 0;
    double PosKd = 0;
    private double PosLastError = 0;

    ElapsedTime PIDtimer = new ElapsedTime();

    public double WDLength = 9.133858, WDWidth = 9.763780, centerRadius = Math.sqrt(WDLength * WDLength + WDWidth * WDWidth);

    public double encoderTicksPerMotorRotation = 1, encoderTicksPerServoRotation = 8192, motorRotationsPerWheelRotation = 1; // needs to be tuned

    public double servoDegreesOfError = 1; // increase this if wheels are twitching back and forth. :O

    public AutonHardware(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        RF = hardwareMap.get(DcMotorEx.class, "rfm"); // RF Encoder
        RB = hardwareMap.get(DcMotorEx.class, "rbm"); // RB Encoder
        LF = hardwareMap.get(DcMotorEx.class, "lfm"); // LF Encoder
        LB = hardwareMap.get(DcMotorEx.class, "lbm"); // LB Encoder
        LA = hardwareMap.get(DcMotorEx.class, "la"); //Left Arm
        RA = hardwareMap.get(DcMotorEx.class, "ra"); //Right Arm

        RFServo = hardwareMap.get(CRServo.class, "rfs");
        RBServo = hardwareMap.get(CRServo.class, "rbs");
        LFServo = hardwareMap.get(CRServo.class, "lfs");
        LBServo = hardwareMap.get(CRServo.class, "lbs");


        LF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LA.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RA.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LA.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        RA.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("Status: ", "Robot Hardware Initialized");
        telemetry.update();
        this.map = hardwareMap;

    }
}

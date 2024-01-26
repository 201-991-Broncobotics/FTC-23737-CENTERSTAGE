package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware {

    public final HardwareMap map;
    public final Telemetry telemetry;


    public final BNO055IMU imu;


    public final DcMotorEx RF, RB, LF, LB, LA, RA;


    public final CRServo RFServo, RBServo, LFServo, LBServo;

    public Servo DServo;


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

    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry) {
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
        DServo = hardwareMap.get(Servo.class, "drone"); //Drone Servo


        RFServo.setDirection(CRServo.Direction.FORWARD);
        RBServo.setDirection(CRServo.Direction.REVERSE);
        LFServo.setDirection(CRServo.Direction.FORWARD);
        LBServo.setDirection(CRServo.Direction.REVERSE);


        LF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LA.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RA.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        LA.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        RA.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("Status: ", "Robot Hardware Initialized");
        telemetry.update();
        this.map = hardwareMap;


    } // initializes everything


    // finds the smallest difference between two angles or wraps an angle to between -180 and 180 when target is 0 (when wrapAngle is 360)
    // wrapAngle of 180 treats the targetAngle and the angle opposite of targetAngle the same
    public double angleDifference(double currentAngle, double targetAngle, int wrapAngle) {
        double result1 = Math.floorMod(Math.round((targetAngle - currentAngle) * 100), wrapAngle * 100L) * 0.01;
        double result2 = Math.floorMod(Math.round((targetAngle - currentAngle) * 100), -wrapAngle * 100L) * 0.01;
        if (Math.abs(result1) <= Math.abs(result2)) return result1;
        else return result2;
    }



    public void methodSleep(long time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            // Wait the set amount of time in milliseconds while in a method
        }
    }


    public void driveSwerveWithControllers(double strafe, double forward, double turn, double throttle) {
        double A = strafe - turn * (WDLength / centerRadius);
        double B = strafe + turn * (WDLength / centerRadius);
        double C = forward - turn * (WDLength / centerRadius);
        double D = forward + turn * (WDLength / centerRadius);
        double RFPower = Math.sqrt(B * B + C * C);
        double LFPower = Math.sqrt(B * B + D * D);
        double LBPower = Math.sqrt(A * A + D * D);
        double RBPower = Math.sqrt(A * A + C * C);
        double max_power = Math.max(1, Math.max(Math.max(RFPower, LFPower), Math.max(LBPower, RBPower))); // keeps all motor powers under 1
        RFPower = RFPower / max_power; // target motor speeds
        LFPower = LFPower / max_power;
        LBPower = LBPower / max_power;
        RBPower = RBPower / max_power;
        double RFAngle = Math.toDegrees(Math.atan2(B, C)); // Target wheel angles
        double LFAngle = Math.toDegrees(Math.atan2(B, D));
        double LBAngle = Math.toDegrees(Math.atan2(A, D));
        double RBAngle = Math.toDegrees(Math.atan2(A, C));

        // find current angle in degrees of the swerve wheel and wrap it to between -180 and 180
        double currentRFPosition = angleDifference(RF.getCurrentPosition() / encoderTicksPerServoRotation * 360, 0, 360);
        double currentLFPosition = angleDifference(LF.getCurrentPosition() / encoderTicksPerServoRotation * 360, 0, 360);
        double currentLBPosition = angleDifference(LB.getCurrentPosition() / encoderTicksPerServoRotation * 360, 0, 360);
        double currentRBPosition = angleDifference(RB.getCurrentPosition() / encoderTicksPerServoRotation * 360, 0, 360);

        // move servos in direction of target angle or target angle + 180 depending on which one is closer
        // unless the change in angle is less than the error range
        double RFServoTurnDistance = angleDifference(currentRFPosition, RFAngle, 180);
        double LFServoTurnDistance = angleDifference(currentLFPosition, LFAngle, 180);
        double LBServoTurnDistance = angleDifference(currentLBPosition, LBAngle, 180);
        double RBServoTurnDistance = angleDifference(currentRBPosition, RBAngle, 180);
        double RFServoPower = 0;
        double LFServoPower = 0;
        double LBServoPower = 0;
        double RBServoPower = 0;
        if (RFPower > 0) {
            if (RFServoTurnDistance > servoDegreesOfError) RFServoPower = -1;
            else if (RFServoTurnDistance < -servoDegreesOfError) RFServoPower = 1;
        }
        if (LFPower > 0) {
            if (LFServoTurnDistance > servoDegreesOfError) LFServoPower = -1;
            else if (LFServoTurnDistance < -servoDegreesOfError) LFServoPower = 1;
        }
        if (LBPower > 0) {
            if (LBServoTurnDistance > servoDegreesOfError) LBServoPower = -1;
            else if (LBServoTurnDistance < -servoDegreesOfError) LBServoPower = 1;
        }
        if (RBPower > 0) {
            if (RBServoTurnDistance > servoDegreesOfError) RBServoPower = -1;
            else if (RBServoTurnDistance < -servoDegreesOfError) RBServoPower = 1;
        }

        // set all servo powers at basically the same time
        RFServo.setPower(RFServoPower);
        LFServo.setPower(LFServoPower);
        LBServo.setPower(LBServoPower);
        RBServo.setPower(RBServoPower);

        // if the difference between current angle and target angle is greater than 90, move motor in reverse
        if (Math.abs(angleDifference(currentRFPosition, RFAngle, 360)) > 90)
            RF.setPower(throttle * -RFPower);
        else RF.setPower(throttle * RFPower);
        if (Math.abs(angleDifference(currentLFPosition, LFAngle, 360)) > 90)
            LF.setPower(throttle * -LFPower);
        else LF.setPower(throttle * LFPower);
        if (Math.abs(angleDifference(currentLBPosition, LBAngle, 360)) > 90)
            LB.setPower(throttle * -LBPower);
        else LB.setPower(throttle * LBPower);
        if (Math.abs(angleDifference(currentRBPosition, RBAngle, 360)) > 90)
            RB.setPower(throttle * -RBPower);
        else RB.setPower(throttle * RBPower);


        telemetry.addData("RF:", currentRFPosition);
        telemetry.addData("LF:", currentLFPosition);
        telemetry.addData("LB:", currentLBPosition);
        telemetry.addData("RB:", currentRBPosition);
        telemetry.addData("RFAngle:", RFAngle);
        telemetry.addData("LFAngle:", LFAngle);
        telemetry.addData("LBAngle:", LBAngle);
        telemetry.addData("RBAngle:", RBAngle);
        telemetry.addData("RFPower:", RFPower);
        telemetry.addData("LFPower:", LFPower);
        telemetry.addData("LBPower:", LBPower);
        telemetry.addData("RBPower:", RBPower);
        telemetry.update();
    }


    public double PosPID(double PosReference, double PosState) {
        double PosError = PosReference - PosState;
        PosIntegralSum += PosError * PIDtimer.seconds();
        double PosDerivative = (PosError - PosLastError) / PIDtimer.seconds();
        PosLastError = PosError;

        PIDtimer.reset();

        return (PosError * PosKp) + (PosDerivative * PosKd) + (PosIntegralSum * PosKi);
    }

}

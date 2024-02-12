package org.firstinspires.ftc.teamcode.teleop;

//import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware {

    public final HardwareMap map;
    public final Telemetry telemetry;
    public final DcMotorEx LA, RA;


    public final DcMotor RF, RB, LF, LB;


    public final CRServo RFServo, RBServo, LFServo, LBServo;

    public Servo DServo;


    // Position PID variables -- PID not set to anything right now
    double PosIntegralSum = 0;
    double PosKp = 0;
    double PosKi = 0;
    double PosKd = 0;
    private double PosLastError = 0;
    ElapsedTime PIDtimer = new ElapsedTime();


    public double WDLength = 9.133858, WDWidth = 9.763780, centerRadius = Math.sqrt(WDLength * WDLength + WDWidth * WDWidth);

    public double encoderTicksPerServoRotation = 8192, servoDegreesOfError = 1.5; // increase this if wheels are twitching back and forth. :O


    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;


        RF = hardwareMap.get(DcMotor.class, "rfm"); // RF Encoder
        RB = hardwareMap.get(DcMotor.class, "rbm"); // RB Encoder
        LF = hardwareMap.get(DcMotor.class, "lfm"); // LF Encoder
        LB = hardwareMap.get(DcMotor.class, "lbm"); // LB Encoder
        LA = hardwareMap.get(DcMotorEx.class, "la"); //Left Arm
        RA = hardwareMap.get(DcMotorEx.class, "ra"); //Right Arm

        RFServo = hardwareMap.get(CRServo.class, "rfs");
        RBServo = hardwareMap.get(CRServo.class, "rbs");
        LFServo = hardwareMap.get(CRServo.class, "lfs");
        LBServo = hardwareMap.get(CRServo.class, "lbs");
        DServo = hardwareMap.get(Servo.class, "drone"); //Drone Servo


        RFServo.setDirection(CRServo.Direction.FORWARD);
        RBServo.setDirection(CRServo.Direction.REVERSE);
        LFServo.setDirection(CRServo.Direction.REVERSE);
        LBServo.setDirection(CRServo.Direction.REVERSE);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LA.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RA.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        double A = -strafe + turn * (WDLength / centerRadius);
        double B = -strafe - turn * (WDLength / centerRadius);
        double C = -forward - turn * (WDLength / centerRadius);
        double D = -forward + turn * (WDLength / centerRadius);
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

        if (RFPower > 0 && Math.abs(RFServoTurnDistance) > servoDegreesOfError) RFServoPower = -1 * RFServoTurnDistance / 55.0; // degrees away from target to start slowing down
        if (LFPower > 0 && Math.abs(LFServoTurnDistance) > servoDegreesOfError) LFServoPower = -1 * LFServoTurnDistance / 55.0;
        if (LBPower > 0 && Math.abs(LBServoTurnDistance) > servoDegreesOfError) LBServoPower = -1 * LBServoTurnDistance / 5.0;
        if (RBPower > 0 && Math.abs(RBServoTurnDistance) > servoDegreesOfError) RBServoPower = -1 * RBServoTurnDistance / 5.0;

        // set all servo powers at basically the same time
        RFServo.setPower(RFServoPower);
        LFServo.setPower(LFServoPower);
        LBServo.setPower(LBServoPower);
        RBServo.setPower(RBServoPower);

        // move the motor in reverse if wheel is rotated 180 degrees from target and stop motors if pointing wrong direction
        double RFMotorInput = throttle * RFPower * Math.sin(((Math.abs(angleDifference(currentRFPosition, RFAngle, 360)) / 90) - 1) * Math.PI / 2);
        double LFMotorInput = throttle * LFPower * Math.sin(((Math.abs(angleDifference(currentLFPosition, LFAngle, 360)) / 90) - 1) * Math.PI / 2);
        double LBMotorInput = throttle * LBPower * Math.sin(((Math.abs(angleDifference(currentLBPosition, LBAngle, 360)) / 90) - 1) * Math.PI / 2);
        double RBMotorInput = throttle * RBPower * Math.sin(((Math.abs(angleDifference(currentRBPosition, RBAngle, 360)) / 90) - 1) * Math.PI / 2);
        RF.setPower(RFMotorInput);
        LF.setPower(LFMotorInput);
        LB.setPower(LBMotorInput);
        RB.setPower(RBMotorInput);


        telemetry.addData("RF:", currentRFPosition);
        telemetry.addData("LF:", currentLFPosition);
        telemetry.addData("LB:", currentLBPosition);
        telemetry.addData("RB:", currentRBPosition);
        telemetry.addData("RFServoPower:", RFServoPower);
        telemetry.addData("LFServoPower:", LFServoPower);
        telemetry.addData("LBServoPower:", LBServoPower);
        telemetry.addData("RBServoPower:", RBServoPower);
        telemetry.addData("RFPower:", RFMotorInput);
        telemetry.addData("LFPower:", LFMotorInput);
        telemetry.addData("LBPower:", LBMotorInput);
        telemetry.addData("RBPower:", RBMotorInput);
        telemetry.update();
    }


    public double PosPID(double PosReference, double PosState) { // PID not currently set to anything
        double PosError = PosReference - PosState;
        PosIntegralSum += PosError * PIDtimer.seconds();
        double PosDerivative = (PosError - PosLastError) / PIDtimer.seconds();
        PosLastError = PosError;

        PIDtimer.reset();

        return (PosError * PosKp) + (PosDerivative * PosKd) + (PosIntegralSum * PosKi);
    }
}
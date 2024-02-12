package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Roadrunner.util.Encoder;

@Autonomous(name = "Testing Auton")
public class DriveTesting extends LinearOpMode {

    public AutonHardware robot;
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new AutonHardware(hardwareMap, telemetry);

        waitForStart();

        MovePID(5,5);
        TurnPID(90,5);
        TurnPID(-90,5);
    }
    public void Move(double p, double distance_in_inches){
        getMotorPositions();
        double wheelDiameter = 3.77953; // in inches
        int countsPerRevolution = 580;

        double wheelCircumference = Math.PI * wheelDiameter;
        double revolutions = distance_in_inches / wheelCircumference;
        int totalCounts = (int) (revolutions * countsPerRevolution);

// Now you can use totalCounts to set your motor target position
        robot.LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LF.setTargetPosition(totalCounts);
        robot.LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.LF.setPower(p); // Set desired motor power
        robot.LB.setPower(p);
        robot.RF.setPower(p);
        robot.RB.setPower(p);


        while (opModeIsActive() && robot.LF.isBusy()) {
            getMotorPositions();
        }

        robot.LF.setPower(0); // Stop motor
        robot.LB.setPower(0);
        robot.RF.setPower(0);
        robot.RB.setPower(0);
    }
    public void MovePID(double target_distance, double tolerance){
        double wheelDiameter = 3.77953; // in inches
        int countsPerRevolution = 580;
        double wheelCircumference = Math.PI * wheelDiameter;
        double targetCounts = (target_distance / wheelCircumference) * countsPerRevolution;

        robot.LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double error = targetCounts - robot.LF.getCurrentPosition();
        double lastError = 0;
        double integral = 0;
        double derivative;
        double kp = 0.1; // Proportional gain. Adjust as needed.
        double ki = 0.01; // Integral gain. Adjust as needed.
        double kd = 0.1; // Derivative gain. Adjust as needed.

        while (Math.abs(error) > tolerance) {
            double currentCounts = robot.LF.getCurrentPosition();
            error = targetCounts - currentCounts;
            integral += error;
            derivative = error - lastError;
            double power = kp * error + ki * integral + kd * derivative;

            robot.LF.setPower(power);
            robot.LB.setPower(power);
            robot.RF.setPower(power);
            robot.RB.setPower(power);

            lastError = error;
        }

        robot.LF.setPower(0);
        robot.LB.setPower(0);
        robot.RF.setPower(0);
        robot.RB.setPower(0);
    }

    public Integer getMotorPositions(){
        int motorPOS = robot.LF.getCurrentPosition();
        telemetry.addData("LF Position: ", motorPOS);
        telemetry.update();
        return motorPOS;
    }
    public Double getCurrentAngle(){
        double currentCount = robot.encoder.getCurrentPosition();
        double currentAngle = (currentCount / 1440.0) * 360.0;
        if (currentAngle > 180) {
            currentAngle -= 360;
        }
        telemetry.addData("Angle: ", currentAngle);
        telemetry.update();
        return currentAngle;
    }
    public void Turn(double p, double target_angle, double tolerance){

        double currentPosition = getCurrentAngle();

        while (Math.abs(currentPosition - target_angle) > tolerance) {

            currentPosition = getCurrentAngle();

            if (currentPosition > target_angle) {
                robot.RFServo.setPower(-p);
                robot.RBServo.setPower(-p);
                robot.LFServo.setPower(-p);
                robot.LBServo.setPower(-p);
            } else {
                robot.RFServo.setPower(p);
                robot.RBServo.setPower(p);
                robot.LFServo.setPower(p);
                robot.LBServo.setPower(p);
            }
            robot.RFServo.setPower(0);
            robot.RBServo.setPower(0);
            robot.LFServo.setPower(0);
            robot.LBServo.setPower(0);
        }
    }
    public void TurnPID(double target_angle, double tolerance){
        double currentPosition = getCurrentAngle();
        double error = target_angle - currentPosition;
        double lastError = 0;
        double integral = 0;
        double derivative;
        double kp = 0.1; // Proportional gain. Adjust as needed.
        double ki = 0.01; // Integral gain. Adjust as needed.
        double kd = 0.1; // Derivative gain. Adjust as needed.

        while (Math.abs(error) > tolerance) {
            currentPosition = getCurrentAngle();
            error = target_angle - currentPosition;
            integral += error;
            derivative = error - lastError;
            double power = kp * error + ki * integral + kd * derivative;

            robot.RFServo.setPower(power);
            robot.RBServo.setPower(power);
            robot.LFServo.setPower(power);
            robot.LBServo.setPower(power);

            lastError = error;
        }

        robot.RFServo.setPower(0);
        robot.RBServo.setPower(0);
        robot.LFServo.setPower(0);
        robot.LBServo.setPower(0);
    }
    }
class AutonHardware { //setting motors to run_to_position for auton

    public final DcMotor RF, RB, LF, LB, LA, RA;
    public final CRServo RFServo, RBServo, LFServo, LBServo;

    public final Encoder encoder;

    // Position PID variables -- PID not set to anything right now
    double PosIntegralSum = 0;
    double PosKp = 0;
    double PosKi = 0;
    double PosKd = 0;
    private double PosLastError = 0;
    ElapsedTime PIDtimer = new ElapsedTime();


    public double WDLength = 9.133858, WDWidth = 9.763780, centerRadius = Math.sqrt(WDLength * WDLength + WDWidth * WDWidth);

    //public double encoderTicksPerServoRotation = 8192, servoDegreesOfError = 1; // increase this if wheels are twitching back and forth. :O


    public AutonHardware(HardwareMap hardwareMap, Telemetry telemetry) {


        RF = hardwareMap.get(DcMotor.class, "rfm"); // RF Encoder
        RB = hardwareMap.get(DcMotor.class, "rbm"); // RB Encoder
        LF = hardwareMap.get(DcMotor.class, "lfm"); // LF Encoder
        LB = hardwareMap.get(DcMotor.class, "lbm"); // LB Encoder
        LA = hardwareMap.get(DcMotor.class, "la"); //Left Arm
        RA = hardwareMap.get(DcMotor.class, "ra"); //Right Arm

        RFServo = hardwareMap.get(CRServo.class, "rfs");
        RBServo = hardwareMap.get(CRServo.class, "rbs");
        LFServo = hardwareMap.get(CRServo.class, "lfs");
        LBServo = hardwareMap.get(CRServo.class, "lbs");

        encoder = hardwareMap.get(Encoder.class, "encoder");


        RFServo.setDirection(CRServo.Direction.REVERSE);
        RBServo.setDirection(CRServo.Direction.REVERSE);
        LFServo.setDirection(CRServo.Direction.REVERSE);
        LBServo.setDirection(CRServo.Direction.REVERSE);

        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);
        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.FORWARD);

        LA.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RA.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        LA.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        RA.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        encoder.setDirection(Encoder.Direction.FORWARD);


        telemetry.addData("Status: ", "Robot Hardware Initialized");
        telemetry.update();


    } // initializes everything
    public double PosPID(double PosReference, double PosState) { // PID not currently set to anything
        double PosError = PosReference - PosState;
        PosIntegralSum += PosError * PIDtimer.seconds();
        double PosDerivative = (PosError - PosLastError) / PIDtimer.seconds();
        PosLastError = PosError;

        PIDtimer.reset();

        return (PosError * PosKp) + (PosDerivative * PosKd) + (PosIntegralSum * PosKi);
    }
    public void methodSleep(long time){
        try {
            Thread.sleep(time * 1000);
        } catch (InterruptedException e) {
        }
    }
}
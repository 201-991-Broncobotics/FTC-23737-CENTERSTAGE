package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Roadrunner.util.Encoder;

@Autonomous(name = "Looking for Prop")
public class Auton23737 extends CommandOpMode {

    AutonHardwareMain robot = new AutonHardwareMain(hardwareMap, telemetry);
    Camera hy = new Camera(hardwareMap);
    DSensor ds = new DSensor(hardwareMap);
    @Override
    public void initialize() {

        waitForStart();

        while (opModeIsActive()) {

            Move(0.25,10);
            ds.getDsResultOne();
            ds.getDsResultTwo();
            ds.getDsResultThree();
            ds.getComparedDSOne();
            ds.getComparedDSTwo();
            ds.getComparedDSThree();
            if (ds.comparedDSOne == 0){
                telemetry.addLine("Prop Found in Front");
                telemetry.update();
            } else if (ds.comparedDSTwo == 0){
                telemetry.addLine("Prop Found to Right");
                telemetry.update();
                Turn(0.2,90,5);
            } else if (ds.comparedDSThree == 0){
                telemetry.addLine("Prop Found to Left");
                telemetry.update();
                Turn(0.2,-90,5);
            }
        }
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
    public Integer getMotorPositions(){
        int motorPOS = robot.LF.getCurrentPosition();
        telemetry.addData("LF Position: ", motorPOS);
        telemetry.update();
        return motorPOS;
    }
    public Double getCurrentAngle(){
        double currentCount = robot.encoder.getCurrentPosition();
        double currentAngle = (currentCount / 1440.0) * 360.0;
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
}
class AutonHardwareMain { //setting motors to run_to_position for auton

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


    public AutonHardwareMain(HardwareMap hardwareMap, Telemetry telemetry) {


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

    public void methodSleep(long time) {
        try {
            Thread.sleep(time * 1000);
        } catch (InterruptedException e) {
        }
    }
}





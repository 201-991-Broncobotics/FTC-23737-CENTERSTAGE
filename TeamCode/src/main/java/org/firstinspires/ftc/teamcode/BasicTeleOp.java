package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name="TeleOp", group="Iterative Opmode")
public class BasicTeleOp extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);

        Gamepad driver = gamepad1, operator = gamepad2;



        waitForStart();



        while (opModeIsActive() && !isStopRequested()) {




            robot.driveSwerveWithControllers(Math.abs(gamepad1.right_stick_x) * gamepad1.right_stick_x,
                    Math.abs(gamepad1.right_stick_y) * gamepad1.right_stick_y,
                    (Math.abs(gamepad1.left_stick_x) * gamepad1.left_stick_x) * (1 - 0.4 * gamepad1.left_trigger),
                    1 - 0.6 * gamepad1.left_trigger);


            telemetry.update();
        }
    }
}


class RobotHardware {

    public final HardwareMap map;
    public final Telemetry telemetry;


    public final BNO055IMU imu;


    public final DcMotorEx RF, RB, LF, LB;


    public final CRServo RF1, RF2, RB1, RB2, LF1, LF2, LB1, LB2;



    // VFB Position PID variables
    double PosIntegralSum = 0;
    double PosKp = 0;
    double PosKi = 0;
    double PosKd = 0;
    private double PosLastError = 0;

    ElapsedTime PIDtimer = new ElapsedTime();

    double WDLength = 9.133858, WDWidth = 9.763780, wheelDiameter = 3.779528, centerRadius = Math.sqrt(WDLength * WDLength + WDWidth * WDWidth);

    double encoderTicksPerMotorRotation = 1, encoderTicksPerServoRotation = 1, motorRotationsPerWheelRotation = 1; // needs to be tuned

    double servoDegreesOfError = 1; // increase this if wheels are twitching back and forth

    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry){
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


        RF = hardwareMap.get(DcMotorEx.class, "RF"); // RF Encoder
        RB = hardwareMap.get(DcMotorEx.class, "RB"); // RB Encoder
        LF = hardwareMap.get(DcMotorEx.class, "LF"); // LF Encoder
        LB = hardwareMap.get(DcMotorEx.class, "LB"); // LB Encoder

        RF1 = hardwareMap.get(CRServo.class, "RF1");
        RF2 = hardwareMap.get(CRServo.class, "RF2");
        RB1 = hardwareMap.get(CRServo.class, "RB1");
        RB2 = hardwareMap.get(CRServo.class, "RB2");
        LF1 = hardwareMap.get(CRServo.class, "LF1");
        LF2 = hardwareMap.get(CRServo.class, "LF2");
        LB1 = hardwareMap.get(CRServo.class, "LB1");
        LB2 = hardwareMap.get(CRServo.class, "LB2");


        LF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("Status", "Robot Hardware Initialized");
        telemetry.update();
        this.map = hardwareMap;


    } // initializes everything



    public double angleDifference(double CurrentAngle, double TargetAngle, int wrapAngle) {
        double result1 = Math.floorMod(Math.round((TargetAngle - CurrentAngle) * 100), wrapAngle * 100L) * 0.01;
        double result2 = Math.floorMod(Math.round((TargetAngle - CurrentAngle) * 100), -wrapAngle * 100L) * 0.01;
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
        double max_power = Math.max(1, Math.max(Math.max(RFPower, LFPower), Math.max(LBPower, RBPower)));
        RFPower = RFPower / max_power; // prevent motor power from being over 1
        LFPower = LFPower / max_power;
        LBPower = LBPower / max_power;
        RBPower = RBPower / max_power;
        double RFAngle = Math.atan2(B, C);
        double LFAngle = Math.atan2(B, D);
        double LBAngle = Math.atan2(A, D);
        double RBAngle = Math.atan2(A, C);

        // find current angle in degrees of the swerve wheel and puts it between -180 and 180
        double currentRFPosition = angleDifference(RF.getCurrentPosition() / encoderTicksPerMotorRotation, 0, 360);
        double currentLFPosition = angleDifference(LF.getCurrentPosition() / encoderTicksPerMotorRotation, 0, 360);
        double currentLBPosition = angleDifference(LB.getCurrentPosition() / encoderTicksPerMotorRotation, 0, 360);
        double currentRBPosition = angleDifference(RB.getCurrentPosition() / encoderTicksPerMotorRotation, 0, 360);

        // move servos in direction of target angle or target angle + 180 depending on which one is closer
        // unless the change in angle is less than error range
        if (RFPower > 0) {
            if (angleDifference(currentRFPosition, RFAngle, 180) >= servoDegreesOfError) {
                RF1.setPower(1);
                RF2.setPower(1);
            } else if (angleDifference(currentRFPosition, RFAngle, 180) <= -servoDegreesOfError) {
                RF1.setPower(-1);
                RF2.setPower(-1);
            }
        } if (LFPower > 0) {
            if (angleDifference(currentLFPosition, LFAngle, 180) >= servoDegreesOfError) {
                LF1.setPower(1);
                LF2.setPower(1);
            } else if (angleDifference(currentLFPosition, LFAngle, 180) <= -servoDegreesOfError) {
                LF1.setPower(-1);
                LF2.setPower(-1);
            }
        } if (LBPower > 0) {
            if (angleDifference(currentLBPosition, LBAngle, 180) >= servoDegreesOfError) {
                LB1.setPower(1);
                LB2.setPower(1);
            } else if (angleDifference(currentLBPosition, LBAngle, 180) <= -servoDegreesOfError) {
                LB1.setPower(-1);
                LB2.setPower(-1);
            }
        } if (RBPower > 0) {
            if (angleDifference(currentRBPosition, RBAngle, 180) >= servoDegreesOfError) {
                RB1.setPower(1);
                RB2.setPower(1);
            } else if (angleDifference(currentRBPosition, RBAngle, 180) <= -servoDegreesOfError) {
                RB1.setPower(-1);
                RB2.setPower(-1);
            }
        }

        // if the difference between current angle and target angle is greater than 180, move motor in reverse
        if (Math.abs(angleDifference(currentRFPosition, RFAngle, 360)) > 180) RF.setPower(throttle * -RFPower);
        else RF.setPower(throttle * RFPower);
        if (Math.abs(angleDifference(currentLFPosition, LFAngle, 360)) > 180) LF.setPower(throttle * -LFPower);
        else LF.setPower(throttle * LFPower);
        if (Math.abs(angleDifference(currentLBPosition, LBAngle, 360)) > 180) LB.setPower(throttle * -LBPower);
        else LB.setPower(throttle * LBPower);
        if (Math.abs(angleDifference(currentRBPosition, RBAngle, 360)) > 180) RB.setPower(throttle * -RBPower);
        else RB.setPower(throttle * RBPower);

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

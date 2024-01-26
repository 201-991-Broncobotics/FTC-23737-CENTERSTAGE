package org.firstinspires.ftc.teamcode.teleop;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;


import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name="TeleOp", group="Iterative Opmode")
public class BasicTeleOp extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);

        Gamepad driver = gamepad1, operator = gamepad2;

        ElapsedTime mRuntime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double LastTime = mRuntime.time();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad2.dpad_left){
                robot.DServo.setPosition(.5);
            }

            if (gamepad2.left_bumper){
                robot.LA.setPower(-1);
                robot.RA.setPower(-1);
            } else if (gamepad2.right_bumper){
                robot.LA.setPower(1);
                robot.RA.setPower(1);
            } else {
                robot.LA.setPower(0);
                robot.RA.setPower(0);
            }


            robot.driveSwerveWithControllers(
                    Math.abs(gamepad1.right_stick_x) * gamepad1.right_stick_x,
                    Math.abs(gamepad1.right_stick_y) * gamepad1.right_stick_y,
                    (Math.abs(gamepad1.left_stick_x) * gamepad1.left_stick_x) * (1 - 0.4 * gamepad1.left_trigger),
                    1 - 0.6 * gamepad1.left_trigger * (1 - gamepad1.right_trigger)); // right trigger stops motors but still lets servos spin


            telemetry.addData("FPS:", Math.round((1 / (mRuntime.time() - LastTime)) * 1000));
            LastTime = mRuntime.time();
            //telemetry.update(); // telemetry is temporary updating in swerve drive method to make sure telemetry shows up
        }
    }
}



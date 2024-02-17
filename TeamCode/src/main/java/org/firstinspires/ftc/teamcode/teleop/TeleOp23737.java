package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "TeleOp23737")
public class TeleOp23737 extends LinearOpMode { //Has all mechanisms for when we have an outtake

    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);

        Gamepad driver = gamepad1, operator = gamepad2;

        ElapsedTime mRuntime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double LastTime = mRuntime.time();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad2.dpad_left) {
                robot.DServo.setPosition(.25);
                robot.methodSleep(2);
                robot.DServo.setPosition(0);
            }
            if (gamepad2.left_bumper) {
                robot.LOServo.setPower(0.75);
                robot.ROServo.setPower(0.75);
            } else {
                robot.LOServo.setPower(-0.75);
                robot.ROServo.setPower(-0.75);
                robot.methodSleep(3);
                robot.LOServo.setPower(0);
                robot.ROServo.setPower(0);
            }
            if (gamepad2.left_trigger > 0.05) {
                double power = gamepad2.left_trigger;
                robot.LA.setPower(power);
                robot.RA.setPower(power);
                if (power > 0.75){
                    robot.LA.setPower(1);
                    robot.RA.setPower(1);
                }
            } else if (gamepad2.right_trigger > 0.05) {
                double power1 = gamepad2.right_trigger;
                robot.LA.setPower(-power1);
                robot.RA.setPower(-power1);
                if (power1 > 0.75){
                    robot.LA.setPower(-1);
                    robot.RA.setPower(-1);
                }
            } else {
                robot.LA.setPower(0);
                robot.RA.setPower(0);
            }


            robot.driveSwerveWithControllers(
                    Math.abs(gamepad1.left_stick_x) * gamepad1.left_stick_x,
                    Math.abs(gamepad1.left_stick_y) * gamepad1.left_stick_y,
                    (Math.abs(gamepad1.right_stick_x) * gamepad1.right_stick_x) * (1 - 0.4 * gamepad1.left_trigger),
                    (1 - 0.6 * gamepad1.left_trigger) * (1 - gamepad1.right_trigger)); // right trigger stops motors but still lets servos spin


            telemetry.addData("FPS:", Math.round((1 / (mRuntime.time() - LastTime)) * 1000));
            telemetry.addData("Throttle:", 1 - 0.6 * gamepad1.left_trigger * (1 - gamepad1.right_trigger));
            LastTime = mRuntime.time();
        }
    }
}





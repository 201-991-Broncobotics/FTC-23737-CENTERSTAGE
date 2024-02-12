package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "DONT USE UNTIL OUTTAKE IS CONFIGURED!!!")
public class TeleOp23737 extends LinearOpMode { //Has all mechanisms for when we have an outtake

    Servo OLServo, ORServo;

    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);
        OLServo = hardwareMap.get(Servo.class, "leftouttake");
        ORServo = hardwareMap.get(Servo.class, "rightouttake");

        Gamepad driver = gamepad1, operator = gamepad2;

        ElapsedTime mRuntime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double LastTime = mRuntime.time();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad2.dpad_up) {
                robot.DServo.setPosition(.25);
                robot.methodSleep(1500);
                robot.DServo.setPosition(0);
            }
            if (gamepad2.dpad_left){
                OLServo.getPosition();
                OLServo.setPosition(0.083);
            } else if (gamepad2.dpad_right){
                ORServo.getPosition();
                ORServo.setPosition(0.083);
            } else if (gamepad2.x){
                OLServo.getPosition();
                OLServo.setPosition(0);
            } else if (gamepad2.b){
                ORServo.getPosition();
                ORServo.setPosition(0);
            }
            if (gamepad2.left_bumper) {
                robot.LA.setPower(0.25);
                robot.RA.setPower(0.25);
                robot.RA.setTargetPosition(575);
            } else if (gamepad2.right_bumper) {
                robot.LA.setPower(0.25);
                robot.RA.setPower(0.25);
                robot.RA.setTargetPosition(0);
            } else {
                robot.LA.setPower(0);
                robot.RA.setPower(0);
            }


            robot.driveSwerveWithControllers(
                    Math.abs(gamepad1.right_stick_x) * gamepad1.right_stick_x,
                    Math.abs(gamepad1.left_stick_y) * gamepad1.left_stick_y,
                    (Math.abs(gamepad1.left_stick_x) * gamepad1.left_stick_x) * (1 - 0.4 * gamepad1.left_trigger),
                    (1 - 0.6 * gamepad1.left_trigger) * (1 - gamepad1.right_trigger)); // right trigger stops motors but still lets servos spin


            telemetry.addData("FPS:", Math.round((1 / (mRuntime.time() - LastTime)) * 1000));
            telemetry.addData("Throttle:", 1 - 0.6 * gamepad1.left_trigger * (1 - gamepad1.right_trigger));
            LastTime = mRuntime.time();
            //telemetry.update(); // telemetry is temporary updating in swerve drive method to make sure telemetry shows up
        }
    }
}





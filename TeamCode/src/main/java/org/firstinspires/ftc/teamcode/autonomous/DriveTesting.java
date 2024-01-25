package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.teleop.RobotHardware;

@Autonomous(name = "Testing Auton")
public class DriveTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            new MoveRobot(10,0,0);
        }
    }
}

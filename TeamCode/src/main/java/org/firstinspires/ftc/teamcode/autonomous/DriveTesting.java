package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.autonomous.AutonFunctions;
@Autonomous(name = "Testing Auton")
public class DriveTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        AutonHardware robot = new AutonHardware(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
        }
    }
}

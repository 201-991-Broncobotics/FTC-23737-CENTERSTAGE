package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/*I CAN EXPLAIN!!!!!
So basically because we can't get dead wheels (for some reason) and we're using CRServos which makes it REALLY hard
to be accurate with the angles, I'm gonna just make the robot tankDrive while in Auton
*/
public class TankDrive extends SubsystemBase {

    HardwareMap map;
    Telemetry telemetry;
    AutonHardware robot;

    public TankDrive(HardwareMap map, Telemetry telemetry){
        this.map = map;
        this.telemetry = telemetry;
        robot = new AutonHardware(map, telemetry);
    }
}

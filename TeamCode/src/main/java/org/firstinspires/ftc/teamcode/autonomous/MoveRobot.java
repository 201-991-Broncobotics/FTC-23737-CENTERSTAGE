package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.RobotHardware;
public class MoveRobot extends CommandBase {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    AutonHardware robot = new AutonHardware(hardwareMap,telemetry);

    final double target_x, target_y, target_angle;

    public MoveRobot(int target_x, int target_y, int target_angle){
        this.target_x = target_x;
        this.target_y = target_y;
        this.target_angle = target_angle;
    }
    @Override
    public void initialize(){
        robot.RFServo.setPower(target_angle);
    }
    @Override
    public void execute(){

    }
    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){

        return true;
    }
}

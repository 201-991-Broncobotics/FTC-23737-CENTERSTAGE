package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutonFunctions {

    HardwareMap map;
    Telemetry telemetry;
    AutonHardware robot = new AutonHardware(map, telemetry);
    public double[] Errors;
    public double[] getErrors(){
        Errors = new double[4];
        Errors[1] = robot.RF.getTargetPosition() - robot.RF.getCurrentPosition();
        Errors[2] = robot.RB.getTargetPosition() - robot.RB.getCurrentPosition();
        Errors[3] = robot.LF.getTargetPosition() - robot.LF.getCurrentPosition();
        Errors[4] = robot.LB.getTargetPosition() - robot.LB.getCurrentPosition();
        return Errors;
    }
    public void correctMotors(){
        getErrors();
        if (Errors[1] == 0) {
            robot.RF.setPower(0);
        } else {
            robot.RF.isBusy();
        }
        if (Errors[2] == 0){
            robot.RB.setPower(0);
        } else {
            robot.RB.isBusy();
        }
        if (Errors[3] == 0) {
            robot.LF.setPower(0);
        } else {
            robot.LF.isBusy();
        }
        if (Errors[4] == 0) {
            robot.LB.setPower(0);
        } else {
            robot.LB.isBusy();
        }
    }
    public void halfTurn(){ //Turns the servos 90 degrees
    }
}


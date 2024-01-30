package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.lang.Math;
/*Waffles video showed FOUR distance Sensors surrounding every part of the robot
In the case we need more distance sensors just add them...
*/

public class DSensor {
    DistanceSensor dsf; //Front
    DistanceSensor dsl; //Left
    DistanceSensor dsr; //Right
    Telemetry telemetry;
    public double dsResultOne; //Front
    public double dsResultTwo; //Left
    public double dsResultThree; //Right
    public double minDistance;
    public double comparedDSOne;
    public double comparedDSTwo;
    public double comparedDSThree;



    public DSensor(HardwareMap map) {
        dsf = map.get(DistanceSensor.class, "dsensorfront");
        dsl = map.get(DistanceSensor.class, "dsensorleft");
        dsr = map.get(DistanceSensor.class, "dsensorright");
    }
    public double getDsResultOne(){
        dsResultOne = dsf.getDistance(DistanceUnit.INCH);
        telemetry.addData("Distance Sensor Read: ", dsResultOne);
        return dsResultOne;
    }
    public double getDsResultTwo(){
        dsResultTwo = dsl.getDistance(DistanceUnit.INCH);
        telemetry.addData("Distance Sensor Read: ", dsResultTwo);
        return dsResultTwo;
    }
    public double getDsResultThree(){
        dsResultThree = dsr.getDistance(DistanceUnit.INCH);
        telemetry.addData("Distance Sensor Read: ", dsResultThree);
        return dsResultThree;
    }
    public double getMinDistance(){
        double CalcL1 = Math.min(dsResultOne,dsResultTwo);
        minDistance = Math.min(CalcL1,dsResultThree);
        return minDistance;
    }
    public double getComparedDSOne(){
        comparedDSOne = Double.compare(minDistance, dsResultOne);
        return comparedDSOne;
    }
    public double getComparedDSTwo(){
        comparedDSTwo = Double.compare(minDistance, dsResultTwo);
        return comparedDSTwo;
    }
    public double getComparedDSThree(){
        comparedDSThree = Double.compare(minDistance,dsResultThree);
        return comparedDSThree;
    }
    public void turnOff() {
        telemetry.addLine("Distance Sensor Turning Off");
        dsf.close();
        dsl.close();
        dsr.close();
    }
}


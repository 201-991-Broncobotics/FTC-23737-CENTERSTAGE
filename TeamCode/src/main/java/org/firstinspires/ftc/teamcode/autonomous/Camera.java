package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.dfrobot.HuskyLens;


public class Camera {
    //Andrew Was Here And He likes
    HuskyLens camera;
    public boolean PropFound;
    Telemetry telemetry;
    public double blueLength;
    public double redLength;
    public double tagOneLength;
    public double tagTwoLength;
    public double tagThreeLength;
    public double tagFourLength;
    public double tagFiveLength;

    public Camera(HardwareMap map) {
        camera = map.get(HuskyLens.class, "huskylens");
    }

    public void initCamera(boolean isOn) {
        if (isOn) {
            camera.initialize();
            camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
            telemetry.addLine("HuskyLens is Activated!");
        }
    }

    //ALWAYS CALL AT LEAST ONE OF THESE BEFORE CALLING findTag!!!!!!
    public double getTagOneLength() {
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        HuskyLens.Block[] tagOne = camera.blocks(1);
        tagOneLength = tagOne.length;
        return tagOneLength;

    }
    public double getTagTwoLength() {
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        HuskyLens.Block[] tagTwo = camera.blocks(1);
        tagTwoLength = tagTwo.length;
        return tagTwoLength;
    }
    public double getTagThreeLength() {
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        HuskyLens.Block[] tagThree = camera.blocks(1);
        tagThreeLength = tagThree.length;
        return tagThreeLength;
    }
    public double getTagFourLength() {
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        HuskyLens.Block[] tagFour = camera.blocks(1);
        tagFourLength = tagFour.length;
        return tagFourLength;
    }
    public double getTagFiveLength() {
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        HuskyLens.Block[] tagFive = camera.blocks(1);
        tagFiveLength = tagFive.length;
        return tagFiveLength;
    }

    public void findTag() {
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        if (tagOneLength > 2.5) {
            boolean tagOneFound = true;
            telemetry.addLine("Tag One Found!");
        }
        if (tagTwoLength > 2.5) {
            boolean tagTwoFound = true;
            telemetry.addLine("Tag Two Found!");
        }
        if (tagThreeLength > 2.5) {
            boolean tagThreeFound = true;
            telemetry.addLine("Tag Three Found!");
        }
        if (tagFourLength > 2.5) {
            boolean tagFourFound = true;
            telemetry.addLine("Tag Four Found!");
        }
        if (tagFiveLength > 2.5) {
            boolean tagFiveFound = true;
            telemetry.addLine("Tag One Found!");
        }
    }
}



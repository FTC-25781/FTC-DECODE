package Layered.PhysicalLayer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import android.graphics.Color;

public class ColorSensor extends LinearOpMode {
    private com.qualcomm.robotcore.hardware.ColorSensor colorSensor;
    private NormalizedColorSensor normalizedColorSensor;
    private double redValue;
    private double greenValue;
    private double blueValue;
    private double alphaValue;

    @Override
    public void runOpMode(){
        initHardware();
        while(!isStarted()){
            getColor();
            colorTelemetry();
        }
        waitForStart();
        while(opModeIsActive()){
            getColor();
            colorTelemetry();
        }

    }
    public void initHardware(){
        colorSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "colorSensor1");
        normalizedColorSensor = (NormalizedColorSensor) colorSensor;
    }
    public void teleOpControls(){

    }
    public void getColor(){
        NormalizedRGBA colors = normalizedColorSensor.getNormalizedColors();
        redValue = colors.red;
        greenValue = colors.green;
        blueValue = colors.blue;
        alphaValue = colors.alpha;
    }
    public void colorTelemetry(){
        telemetry.addData("Red Value", "%.2f", redValue);
        telemetry.addData("Green Value", "%.2f", greenValue);
        telemetry.addData("Blue Value", "%.2f", blueValue);
        telemetry.addData("Alpha Value", "%.2f", alphaValue);
        telemetry.update();
    }
    public void hsvValues(){
        float[] hsvValues = new float[3];
        Color.RGBToHSV((int) (redValue * 255), (int) (greenValue * 255), (int) (blueValue * 255), hsvValues);
        float hue = hsvValues[0];
        float saturation = hsvValues[1];
        float value = hsvValues[2];
    }
}

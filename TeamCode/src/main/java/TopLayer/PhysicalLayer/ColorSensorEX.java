package TopLayer.PhysicalLayer;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import android.graphics.Color;

@Disabled
@TeleOp(group="examples")
public class ColorSensorEX extends LinearOpMode {

    private ColorSensor colorSensor;
    private NormalizedColorSensor normalizedColorSensor;
    private double redValue;
    private double greenValue;
    private double blueValue;
    private double alphaValue; // light intensity

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
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor1");
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

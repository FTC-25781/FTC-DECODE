package TopLayer.PhysicalLayer;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;

public class ServoExtension implements Servo {

    ServoImpl temp;
    ServoController test;


    // create constructor
    public ServoExtension(ServoImpl temp, ServoController test) {
        this.temp = temp;
    }

    @Override
    public ServoController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setDirection(Direction direction) {

    }

    @Override
    public Direction getDirection() {
        return null;
    }

    public void setPosition(int angle, boolean radian) {

    }

    @Override
    public void setPosition(double position) {

    }

    @Override
    public double getPosition() {

        return 0;
    }

    @Override
    public void scaleRange(double min, double max) {

    }

   public double getPower() {

   }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return "";
    }

    @Override
    public String getConnectionInfo() {
        return "";
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}

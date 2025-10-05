package Layered.PhysicalLayer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/**
 * Custom Motor wrapper class for FTC layering.
 * Provides abstraction for speed, voltage capping, and RPM measurement.
 */
public class Motor implements DcMotor {

    private final DcMotorEx motor;              // underlying FTC motor
    private final VoltageSensor voltageSensor; // reference for battery voltage
    private int voltageCap = 12;              // default voltage cap
    private double lastPower = 0.0;           // track last applied power

    // constructor
    public Motor(DcMotorEx motor, VoltageSensor voltageSensor) {
        this.motor = motor;
        this.voltageSensor = voltageSensor;
    }

    // set speed as percentage (0-100)
    public void setSpeed(int speed) {
        double cappedPower = Math.max(-100, Math.min(100, speed)) / 100.0;
        setPower(cappedPower);
    }

    // cap max voltage output (scales motor power)
    public void setVoltageCap(int voltage) {
        this.voltageCap = voltage;
    }

    // returns capped voltage
    public double getVoltage() {
        return (voltageSensor != null) ? voltageSensor.getVoltage() : 0.0;
    }

    // estimate RPM from ticks per second
    public int getRPM() {
        MotorConfigurationType type = motor.getMotorType();
        double ticksPerRev = type.getTicksPerRev();
        double ticksPerSec = motor.getVelocity();
        return (int) ((ticksPerSec / ticksPerRev) * 60.0);
    }

    /* ---------------- Delegate FTC DcMotor Methods ---------------- */

    @Override
    public void setPower(double power) {
        double batteryVoltage = getVoltage();
        if (batteryVoltage > 0) {
            double scale = (double) voltageCap / batteryVoltage;
            lastPower = Math.max(-1.0, Math.min(1.0, power * scale));
            motor.setPower(lastPower);
        } else {
            motor.setPower(power);
        }
    }

    @Override
    public double getPower() {
        return lastPower;
    }

    @Override
    public MotorConfigurationType getMotorType() { return motor.getMotorType(); }

    @Override
    public void setMotorType(MotorConfigurationType motorType) { motor.setMotorType(motorType); }

    @Override
    public DcMotorController getController() { return motor.getController(); }

    @Override
    public int getPortNumber() { return motor.getPortNumber(); }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) { motor.setZeroPowerBehavior(zeroPowerBehavior); }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() { return motor.getZeroPowerBehavior(); }

    @Override
    public void setPowerFloat() { motor.setPowerFloat(); }

    @Override
    public boolean getPowerFloat() { return motor.getPowerFloat(); }

    @Override
    public void setTargetPosition(int position) { motor.setTargetPosition(position); }

    @Override
    public int getTargetPosition() { return motor.getTargetPosition(); }

    @Override
    public boolean isBusy() { return motor.isBusy(); }

    @Override
    public int getCurrentPosition() { return motor.getCurrentPosition(); }

    @Override
    public void setMode(RunMode mode) { motor.setMode(mode); }

    @Override
    public RunMode getMode() { return motor.getMode(); }

    @Override
    public void setDirection(Direction direction) { motor.setDirection(direction); }

    @Override
    public Direction getDirection() { return motor.getDirection(); }

    @Override
    public Manufacturer getManufacturer() { return motor.getManufacturer(); }

    @Override
    public String getDeviceName() { return motor.getDeviceName(); }

    @Override
    public String getConnectionInfo() { return motor.getConnectionInfo(); }

    @Override
    public int getVersion() { return motor.getVersion(); }

    @Override
    public void resetDeviceConfigurationForOpMode() { motor.resetDeviceConfigurationForOpMode(); }

    @Override
    public void close() { motor.close(); }
}

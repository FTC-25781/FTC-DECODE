package Layered.PhysicalLayer;

import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.HashMap;
import java.util.Map;

/**
 * Enhanced SmartServo class for FTC
 * Supports state tracking, angle-based control, calibration, and chamber offset.
 */
public class SmartServo implements com.qualcomm.robotcore.hardware.Servo {

    // ==================== ENUMS ====================
    public enum ServoState {
        IDLE,
        MOVING,
        CALIBRATING,
        ERROR,
        CHAMBER_ADJUSTING
    }

    public enum ServoType {
        STANDARD_180(0.0, 1.0, 180.0),
        CONTINUOUS_360(0.0, 1.0, 360.0),
        HIGH_TORQUE_270(0.0, 1.0, 270.0),
        PRECISION_90(0.0, 1.0, 90.0);

        final double minPosition;
        final double maxPosition;
        final double maxAngleDegrees;

        ServoType(double minPos, double maxPos, double maxAngle) {
            this.minPosition = minPos;
            this.maxPosition = maxPos;
            this.maxAngleDegrees = maxAngle;
        }
    }

    // ==================== INSTANCE VARIABLES ====================
    private final com.qualcomm.robotcore.hardware.Servo servo;
    private ServoState currentState;
    private final ServoType servoType;
    private double targetPosition;
    private double currentAngleDegrees;
    private double chamberAdjustment;
    private Map<String, Double> servoProperties;

    // ==================== CONSTRUCTOR ====================
    public SmartServo(com.qualcomm.robotcore.hardware.Servo servo, ServoType type) {
        this.servo = servo;
        this.servoType = type;
        this.currentState = ServoState.IDLE;
        this.targetPosition = 0.0;
        this.currentAngleDegrees = 0.0;
        this.chamberAdjustment = 0.0;
        initializeServoProperties();
    }

    private void initializeServoProperties() {
        servoProperties = new HashMap<>();
        servoProperties.put("speed", 1.0);
        servoProperties.put("precision", 0.01);
        servoProperties.put("deadband", 0.02);
        servoProperties.put("maxSpeed", 180.0);

        switch (servoType) {
            case HIGH_TORQUE_270:
                servoProperties.put("speed", 0.8);
                break;
            case PRECISION_90:
                servoProperties.put("precision", 0.005);
                break;
        }
    }

    // ==================== ANGLE CONTROL ====================
    public void adjustAngleByDegrees(double degreesChange) {
        setState(ServoState.MOVING);
        double newAngle = currentAngleDegrees + degreesChange;
        newAngle = clamp(newAngle, 0, servoType.maxAngleDegrees);

        double newPosition = degreesToPosition(newAngle);
        newPosition += chamberAdjustment;
        newPosition = clamp(newPosition, 0.0, 1.0); // final safety clamp

        setTargetPosition(newPosition);
        currentAngleDegrees = newAngle;
    }

    public void setAngleDegrees(double degrees) {
        setState(ServoState.MOVING);
        degrees = clamp(degrees, 0, servoType.maxAngleDegrees);

        double position = degreesToPosition(degrees) + chamberAdjustment;
        position = clamp(position, 0.0, 1.0);

        setTargetPosition(position);
        currentAngleDegrees = degrees;
    }

    // ==================== CHAMBER OFFSET ====================
    public void adjustChamber(double chamberOffset) {
        setState(ServoState.CHAMBER_ADJUSTING);

        // Safety limit for chamber offset
        this.chamberAdjustment = clamp(chamberOffset, -0.3, 0.3);

        double adjustedPosition = degreesToPosition(currentAngleDegrees) + chamberAdjustment;
        adjustedPosition = clamp(adjustedPosition, 0.0, 1.0);

        setTargetPosition(adjustedPosition);
        setState(ServoState.IDLE);
    }

    // ==================== POSITION MAPPING ====================
    private double degreesToPosition(double degrees) {
        return degrees / servoType.maxAngleDegrees;
    }

    private double positionToDegrees(double position) {
        return position * servoType.maxAngleDegrees;
    }

    // ==================== MOVEMENT ====================
    public void setTargetPosition(double position) {
        double safePos = clamp(position, 0.0, 1.0);
        this.targetPosition = safePos;
        servo.setPosition(safePos);
        setState(ServoState.IDLE);
    }

    // ==================== STATE ====================
    public void setState(ServoState state) {
        this.currentState = state;
    }

    public ServoState getState() {
        return currentState;
    }

    // ==================== CALIBRATION ====================
    public void calibrate() {
        setState(ServoState.CALIBRATING);
        ElapsedTime timer = new ElapsedTime();

        servo.setPosition(0.0);
        timer.reset();
        while (timer.seconds() < 0.5) { /* non-blocking placeholder */ }

        servo.setPosition(1.0);
        timer.reset();
        while (timer.seconds() < 0.5) { /* non-blocking placeholder */ }

        servo.setPosition(0.5);
        currentAngleDegrees = servoType.maxAngleDegrees / 2;
        setState(ServoState.IDLE);
    }

    // ==================== UTILITIES ====================
    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public boolean isAtTarget() {
        double tolerance = getProperty("precision");
        return Math.abs(getPosition() - targetPosition) < tolerance;
    }

    public double getCurrentAngleDegrees() {
        return currentAngleDegrees;
    }

    public ServoType getServoType() {
        return servoType;
    }

    public double getProperty(String key) {
        return servoProperties.getOrDefault(key, 0.0);
    }

    public void setProperty(String key, double value) {
        servoProperties.put(key, value);
    }

    // ==================== SERVO INTERFACE IMPLEMENTATION ====================
    @Override
    public ServoController getController() {
        return servo.getController();
    }

    @Override
    public int getPortNumber() {
        return servo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        servo.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return servo.getDirection();
    }

    @Override
    public void setPosition(double position) {
        double safePos = clamp(position, 0.0, 1.0);
        setState(ServoState.MOVING);
        servo.setPosition(safePos);
        currentAngleDegrees = positionToDegrees(safePos - chamberAdjustment);
        if (isAtTarget()) {
            setState(ServoState.IDLE);
        }
    }

    @Override
    public double getPosition() {
        return servo.getPosition();
    }

    @Override
    public void scaleRange(double min, double max) {
        servo.scaleRange(min, max);
    }

    @Override
    public Manufacturer getManufacturer() {
        return servo.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return servo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return servo.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return servo.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        servo.resetDeviceConfigurationForOpMode();
        setState(ServoState.IDLE);
        currentAngleDegrees = 0.0;
        chamberAdjustment = 0.0;
    }

    @Override
    public void close() {
        setState(ServoState.IDLE);
        servo.close();
    }
}

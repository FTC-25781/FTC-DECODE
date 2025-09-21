package TopLayer.PhysicalLayer;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import java.util.HashMap;
import java.util.Map;

public class ServoExtension implements Servo {

    // State machine enum for servo states
    public enum ServoState {
        IDLE,
        MOVING,
        CALIBRATING,
        ERROR,
        CHAMBER_ADJUSTING
    }

    // Servo type configurations
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

    // Instance variables
    private Servo servo;
    private ServoState currentState;
    private ServoType servoType;
    private double targetPosition;
    private double currentAngleDegrees;
    private double chamberAdjustment;
    private Map<String, Double> servoProperties;

    // Constructor
    public ServoExtension(Servo servo, ServoType type) {
        this.servo = servo;
        this.servoType = type;
        this.currentState = ServoState.IDLE;
        this.targetPosition = 0.0;
        this.currentAngleDegrees = 0.0;
        this.chamberAdjustment = 0.0;
        initializeServoProperties();
    }

    // Initialize servo properties dictionary
    private void initializeServoProperties() {
        servoProperties = new HashMap<>();
        servoProperties.put("speed", 1.0); // Default speed multiplier
        servoProperties.put("precision", 0.01); // Position precision
        servoProperties.put("deadband", 0.02); // Deadband for position tolerance
        servoProperties.put("maxSpeed", 180.0); // Max degrees per second

        // Type-specific adjustments
        switch (servoType) {
            case HIGH_TORQUE_270:
                servoProperties.put("speed", 0.8); // Slower for high torque
                break;
            case PRECISION_90:
                servoProperties.put("precision", 0.005); // Higher precision
                break;
        }
    }

    // Adjust angle based on degrees input
    public void adjustAngleByDegrees(double degreesChange) {
        setState(ServoState.MOVING);

        double newAngle = currentAngleDegrees + degreesChange;

        // Clamp to servo limits
        newAngle = Math.max(0, Math.min(newAngle, servoType.maxAngleDegrees));

        // Convert degrees to servo position (0.0 to 1.0)
        double newPosition = degreesToPosition(newAngle);

        // Apply chamber adjustment if needed
        newPosition += chamberAdjustment;

        // Clamp final position
        newPosition = Math.max(servoType.minPosition,
                Math.min(newPosition, servoType.maxPosition));

        setTargetPosition(newPosition);
        currentAngleDegrees = newAngle;
    }

    // Set angle directly in degrees
    public void setAngleDegrees(double degrees) {
        setState(ServoState.MOVING);

        // Clamp to servo limits
        degrees = Math.max(0, Math.min(degrees, servoType.maxAngleDegrees));

        double position = degreesToPosition(degrees);
        position += chamberAdjustment;

        // Clamp final position
        position = Math.max(servoType.minPosition,
                Math.min(position, servoType.maxPosition));

        setTargetPosition(position);
        currentAngleDegrees = degrees;
    }

    // Adjust chamber position
    public void adjustChamber(double chamberOffset) {
        setState(ServoState.CHAMBER_ADJUSTING);

        this.chamberAdjustment = chamberOffset;

        // Recalculate current position with chamber adjustment
        double adjustedPosition = degreesToPosition(currentAngleDegrees) + chamberAdjustment;
        adjustedPosition = Math.max(servoType.minPosition,
                Math.min(adjustedPosition, servoType.maxPosition));

        setTargetPosition(adjustedPosition);
        setState(ServoState.IDLE);
    }

    // Convert degrees to servo position
    private double degreesToPosition(double degrees) {
        return degrees / servoType.maxAngleDegrees;
    }

    // Convert servo position to degrees
    private double positionToDegrees(double position) {
        return position * servoType.maxAngleDegrees;
    }

    // Set target position with smooth movement
    public void setTargetPosition(double position) {
        this.targetPosition = position;
        servo.setPosition(position);
        setState(ServoState.IDLE);
    }

    // State management
    public void setState(ServoState state) {
        this.currentState = state;
    }

    public ServoState getState() {
        return currentState;
    }

    // Get current angle in degrees
    public double getCurrentAngleDegrees() {
        return currentAngleDegrees;
    }

    // Get servo type
    public ServoType getServoType() {
        return servoType;
    }

    // Get servo property
    public double getProperty(String key) {
        return servoProperties.getOrDefault(key, 0.0);
    }

    // Set servo property
    public void setProperty(String key, double value) {
        servoProperties.put(key, value);
    }

    // Calibrate servo (find limits and center)
    public void calibrate() {
        setState(ServoState.CALIBRATING);

        // Basic calibration - move to known positions
        servo.setPosition(servoType.minPosition);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        servo.setPosition(servoType.maxPosition);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        servo.setPosition(0.5); // Center position
        currentAngleDegrees = servoType.maxAngleDegrees / 2;

        setState(ServoState.IDLE);
    }

    // Check if servo is at target position
    public boolean isAtTarget() {
        double tolerance = getProperty("precision");
        return Math.abs(getPosition() - targetPosition) < tolerance;
    }

    // Servo interface implementations
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
        // Override to use our enhanced position setting
        setState(ServoState.MOVING);
        servo.setPosition(position);
        currentAngleDegrees = positionToDegrees(position - chamberAdjustment);
        setState(ServoState.IDLE);
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

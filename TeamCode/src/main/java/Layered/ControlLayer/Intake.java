package Layered.ControlLayer;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SRSHub; // Assuming SRSHub is accessible

// IMPORTANT: This class is a Subsystem, so it DOES NOT extend OpMode
public class Intake {

    private DcMotorEx intakeMotor;
    private SRSHub.VL53L5CX tofSensor;
    private SRSHub srsHub;

    public enum INTAKE_STATE {
        IDLE,
        INTAKING,
        FULL,
        REVERSING // Added for Outtake hand-off
    }

    private INTAKE_STATE currentState = INTAKE_STATE.IDLE;
    private INTAKE_STATE lastState = INTAKE_STATE.IDLE;

    private int ballCount = 0;
    private boolean lastSensorBlocked = false;

    // Debouncing for sensor
    private ElapsedTime debounceTimer = new ElapsedTime();
    private final double DEBOUNCE_TIME = 0.2;

    // Timeout for safety
    private ElapsedTime intakeTimer = new ElapsedTime();
    private final double INTAKE_TIMEOUT = 15.0;

    // Constants
    private final double BALL_THRESHOLD_IN = 5.0;
    private final double INTAKE_POWER = 0.8;
    private final double REVERSE_POWER = -0.5; // Power for temporary reverse when firing
    private final int MAX_BALLS = 3;

    // --- Constructor (Replaces OpMode init()) ---
    public Intake(HardwareMap hardwareMap) {
        // Hardware Initialization
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // SRSHub Initialization
        SRSHub.Config config = new SRSHub.Config();
        // Assuming VL53L5CX is your Time-of-Flight sensor class
        tofSensor = new SRSHub.VL53L5CX(SRSHub.VL53L5CX.Resolution.GRID_4x4);
        config.addI2CDevice(2, tofSensor);

        RobotLog.clearGlobalWarningMsg();

        srsHub = hardwareMap.get(SRSHub.class, "srs");
        srsHub.init(config);

        // Timer Reset
        debounceTimer.reset();
        intakeTimer.reset();
    }

    // --- Public update() Method (Replaces OpMode loop()) ---
    // Takes Telemetry and debounced button states from the MainTeleOp
    public void update(Telemetry telemetry, boolean aPressed, boolean bPressed, boolean xPressed, boolean isGamepadRumbling) {
        srsHub.update();

        double avgDistanceInches = 999;
        boolean currentSensorBlocked = false;

        // 1. Sensor Reading Logic
        if (!srsHub.disconnected() && !tofSensor.disconnected) {
            short[] distances = tofSensor.distances;
            if (distances != null && distances.length > 0) {
                double sum = 0;
                int validCount = 0;
                for (short d : distances) {
                    if (d > 0) { // Only use valid readings
                        sum += d;
                        validCount++;
                    }
                }
                if (validCount > 0) {
                    avgDistanceInches = (sum / validCount) / 25.4; // mm to inches
                }
            }
        }

        currentSensorBlocked = avgDistanceInches <= BALL_THRESHOLD_IN;

        // 2. Ball Entry Detection (Intake logic only runs if actively intaking)
        if (currentState == INTAKE_STATE.INTAKING &&
                currentSensorBlocked && !lastSensorBlocked &&
                debounceTimer.seconds() > DEBOUNCE_TIME &&
                ballCount < MAX_BALLS) {

            ballCount++;
            debounceTimer.reset();
            // Gamepad rumble handled by MainTeleOp upon state/count change

            if (ballCount >= MAX_BALLS) {
                currentState = INTAKE_STATE.FULL;
            }
        }
        lastSensorBlocked = currentSensorBlocked;

        // 3. Manual State Control (Uses debounced buttons from MainTeleOp)
        if (aPressed && currentState != INTAKE_STATE.FULL) {
            currentState = INTAKE_STATE.INTAKING;
            intakeTimer.reset();
        }

        if (bPressed) {
            currentState = INTAKE_STATE.IDLE;
        }

        if (xPressed) { // Manual reset
            ballCount = 0;
            currentState = INTAKE_STATE.IDLE;
        }

        // 4. Safety Timeout
        if (currentState == INTAKE_STATE.INTAKING && intakeTimer.seconds() > INTAKE_TIMEOUT) {
            currentState = INTAKE_STATE.IDLE;
            // The MainTeleOp should check if the state changed to IDLE from timeout and rumble
        }

        // 5. State Machine Motor Control
        switch (currentState) {
            case IDLE:
            case FULL:
                intakeMotor.setPower(0);
                break;

            case INTAKING:
                intakeMotor.setPower(INTAKE_POWER);
                break;

            case REVERSING:
                intakeMotor.setPower(REVERSE_POWER); // Assists outtake pusher
                break;
        }

        // 6. One-time action / Rumble coordination (Requires external check)
        if (currentState != lastState && !isGamepadRumbling) {
            // MainTeleOp will check this and trigger a rumble if needed
        }
        lastState = currentState;


        // 7. Telemetry
        telemetry.addData("IN_State", currentState);
        telemetry.addData("IN_Ball Count", ballCount + "/" + MAX_BALLS);
        telemetry.addData("IN_Distance", "%.2f inches", avgDistanceInches);
        telemetry.addData("IN_Motor Power", "%.0f%%", intakeMotor.getPower() * 100);
    }

    // --- Public Control Methods for Outtake ---

    public int getBallCount() {
        return ballCount;
    }

    /**
     * Called by Outtake to initiate the fire/push sequence (prepares the ball)
     */
    public void startOuttakeTransfer() {
        if (ballCount > 0) {
            currentState = INTAKE_STATE.REVERSING;
        }
    }

    /**
     * Called by Outtake after the ball has been physically pushed.
     */
    public void decrementBallCount() {
        if (ballCount > 0) {
            ballCount--;
            // Return to IDLE state to prepare for the next intake/fire sequence
            if (ballCount < MAX_BALLS) {
                currentState = INTAKE_STATE.IDLE;
            }
        }
    }

    public INTAKE_STATE getState() {
        return currentState;
    }

    /**
     * Cleans up hardware on OpMode stop.
     */
    public void cleanup() {
        intakeMotor.setPower(0);
    }
}
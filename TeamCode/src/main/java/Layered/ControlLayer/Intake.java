package Layered.ControlLayer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.SRSHub;

@TeleOp(name="Intake Test", group="Sensor")
public class Intake extends OpMode {

    //private DcMotorEx intakeMotor;
    private SRSHub.VL53L5CX tofSensor;
    private SRSHub srsHub;

    public enum INTAKE_STATE {
        IDLE,
        INTAKING,
        FULL,
        REVERSING
    }

    private INTAKE_STATE currentState = INTAKE_STATE.IDLE;
    private INTAKE_STATE lastState = INTAKE_STATE.IDLE;

    private int ballCount = 0;
    private boolean lastSensorBlocked = false;

    private ElapsedTime debounceTimer = new ElapsedTime();
    private final double DEBOUNCE_TIME = 0.2;

    private ElapsedTime intakeTimer = new ElapsedTime();
    private final double INTAKE_TIMEOUT = 15.0;

    private final double BALL_THRESHOLD_IN = 5.0;
    private final double MIN_DETECTION_DISTANCE_MM = 50;
    private final double MAX_DETECTION_DISTANCE_MM = 127; // 5 inches
    private final double INTAKE_POWER = 0.8;
    private final double REVERSE_POWER = -0.5;
    private final int MAX_BALLS = 3;
    private final double BALL_RADIUS_MM = 40.0; // Adjust to actual ball radius

    private double currentXOffset = 0.0;
    private double currentXOffsetPercent = 0.0;
    private boolean objectDetected = false;
    private double correctedDistanceMm = 0.0;
    private int ballZone = -1;

    @Override
    public void init() {
        // Hardware Initialization
        //intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        //intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // SRSHub Initialization
        SRSHub.Config config = new SRSHub.Config();
        tofSensor = new SRSHub.VL53L5CX(SRSHub.VL53L5CX.Resolution.GRID_4x4);
        config.addI2CDevice(3, tofSensor);

        RobotLog.clearGlobalWarningMsg();

        srsHub = hardwareMap.get(SRSHub.class, "srs");
        srsHub.init(config);

        // Timer Reset
        debounceTimer.reset();
        intakeTimer.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        srsHub.update();

        double avgDistanceInches = 999;
        boolean currentSensorBlocked = false;

        if (!srsHub.disconnected() && !tofSensor.disconnected) {

            short[] distances = tofSensor.distances;

            if (distances != null && distances.length == 16) {
                // Calculate offset first for dynamic weight adjustment
                calculateOffset(distances);

                // Get dynamically adjusted weights
                double[] baseWeights = getAdjustedWeights();

                double weightedSum = 0.0;
                double weightTotal = 0.0;

                for (int i = 0; i < 16; i++) {
                    short d = distances[i];
                    if (d > 0) {
                        double w = baseWeights[i];
                        weightedSum += w * d;
                        weightTotal += w;
                    }
                }
                if (weightTotal > 0) {
                    avgDistanceInches = (weightedSum / weightTotal) / 25.4; // mm → inches
                }

                // Calculate corrected distance with angular and radius compensation
                correctedDistanceMm = calculateCorrectedDistance(distances);
            }
        }

        currentSensorBlocked = avgDistanceInches <= BALL_THRESHOLD_IN;

        if (currentState == INTAKE_STATE.INTAKING &&
                currentSensorBlocked && !lastSensorBlocked &&
                debounceTimer.seconds() > DEBOUNCE_TIME &&
                ballCount < MAX_BALLS) {

            ballCount++;
            debounceTimer.reset();

            if (ballCount >= MAX_BALLS) {
                currentState = INTAKE_STATE.FULL;
            }
        }
        lastSensorBlocked = currentSensorBlocked;

        if (gamepad1.a && currentState != INTAKE_STATE.FULL) {
            currentState = INTAKE_STATE.INTAKING;
            intakeTimer.reset();
        }

        if (gamepad1.b) {
            currentState = INTAKE_STATE.IDLE;
        }

        if (gamepad1.x) {
            ballCount = 0;
            currentState = INTAKE_STATE.IDLE;
        }

        if (currentState == INTAKE_STATE.INTAKING && intakeTimer.seconds() > INTAKE_TIMEOUT) {
            currentState = INTAKE_STATE.IDLE;
        }

        switch (currentState) {
            case IDLE:
            case FULL:
                //intakeMotor.setPower(0);
                break;

            case INTAKING:
                //intakeMotor.setPower(INTAKE_POWER);
                break;

            case REVERSING:
                //intakeMotor.setPower(REVERSE_POWER);
                break;
        }

        lastState = currentState;

        telemetry.addData("IN_State", currentState);
        telemetry.addData("IN_Ball Count", ballCount + "/" + MAX_BALLS);
        telemetry.addData("IN_Distance", "%.2f inches", avgDistanceInches);
        /*
        telemetry.addData("IN_Corrected Dist", "%.1f mm (%.2f in)", correctedDistanceMm, correctedDistanceMm / 25.4);
        telemetry.addData("IN_Ball Zone", ballZone >= 0 ? ballZone : "None");
        telemetry.addData("IN_X Offset", "%.1f%% (%s)", currentXOffsetPercent,
                currentXOffset < -0.5 ? "LEFT" : currentXOffset > 0.5 ? "RIGHT" : "CENTER");
        //telemetry.addData("IN_Motor Power", "%.0f%%", intakeMotor.getPower() * 100);
         */
        telemetry.update();
    }
 
    private double calculateCorrectedDistance(short[] distances) {
        // Find minimum valid distance and its zone
        int minDistance = Integer.MAX_VALUE;
        int minZone = -1;
        int detectionCount = 0;
        int sumDistance = 0;

        for (int i = 0; i < 16; i++) {
            int distance = distances[i];
            if (distance > MIN_DETECTION_DISTANCE_MM && distance < MAX_DETECTION_DISTANCE_MM) {
                if (distance < minDistance) {
                    minDistance = distance;
                    minZone = i;
                }
            }
        }

        if (minZone < 0) {
            ballZone = -1;
            return 999.0;
        }

        ballZone = minZone;

        // Count adjacent zones with similar readings for confidence
        for (int i = 0; i < 16; i++) {
            int distance = distances[i];
            if (Math.abs(distance - minDistance) < 30) { // 30mm tolerance
                detectionCount++;
                sumDistance += distance;
            }
        }

        // Use average if multiple zones detect the ball
        double baseDistance = minDistance;
        if (detectionCount >= 2) {
            baseDistance = (double) sumDistance / detectionCount;
        }

        // Calculate angular offset correction
        int row = minZone / 4;
        int col = minZone % 4;

        // Each zone is ~8° apart from center (45° FoV / 4 zones ≈ 11.25° per zone)
        double offsetX = (col - 1.5) * 11.25; // degrees from center
        double offsetY = (row - 1.5) * 11.25;
        double totalOffsetDeg = Math.sqrt(offsetX * offsetX + offsetY * offsetY);

        // Correct for angular offset (closer zones see ball at angle)
        double angularCorrected = baseDistance * Math.cos(Math.toRadians(totalOffsetDeg));

        // Add ball radius to get center distance
        double centerDistance = angularCorrected + BALL_RADIUS_MM;

        return centerDistance;
    }

    private void calculateOffset(short[] distances){
        double weightedX = 0;
        double totalWeight = 0;
        int activeZones = 0;

        for (int row = 0; row < 4; row++){
            for (int col = 0; col < 4; col++){
                int index = row * 4 + col;
                int distance = distances[index];

                if (distance > MIN_DETECTION_DISTANCE_MM && distance < MAX_DETECTION_DISTANCE_MM) {
                    double weight = 1.0 / (distance) / 100.0; // because distance and weight are inveresely proportional, closer = more weight

                    double x = col - 1.5; // center of 4x4 grid

                    weightedX += x * weight;
                    totalWeight += weight;
                    activeZones++;
                }
            }
        }

        if (totalWeight > 0 && activeZones > 0) {
            objectDetected = true;
            currentXOffset = weightedX / totalWeight;
            currentXOffsetPercent = (currentXOffset / 1.5) * 100;
        } else {
            objectDetected = false;
            currentXOffset = 0.0;
            currentXOffsetPercent = 0.0;
        }
    }

    public double[] getAdjustedWeights() {
        double[] baseWeights = {
                0.01, 0.01, 0.01, 0.01,
                0.01, 0.1, 0.1, 0.01,
                0.01, 1.0, 1.0, 0.01,
                0.01, 0.01, 0.01, 0.01
        };

        if (!objectDetected) {
            return baseWeights;
        }

        if (currentXOffset < -0.5) {
            // object on left so shift weights left
            return new double[] {
                    0.01, 0.01, 0.01, 0.01,
                    0.1,  0.1,  0.01, 0.01,
                    1.0,  1.0,  0.01, 0.01,
                    0.01, 0.01, 0.01, 0.01
            };
        } else if (currentXOffset > 0.5) {
            // object on right so shift weights right
            return new double[] {
                    0.01, 0.01, 0.01, 0.01,
                    0.01, 0.01, 0.1,  0.1,
                    0.01, 0.01, 1.0,  1.0,
                    0.01, 0.01, 0.01, 0.01
            };
        } else {
            return new double[] {
                    0.01, 0.01, 0.01, 0.01,
                    0.01, 0.1,  0.1,  0.01,
                    0.01, 1.0,  1.0,  0.01,
                    0.01, 0.01, 0.01, 0.01
            };
        }
    }

    @Override
    public void stop() {
        //intakeMotor.setPower(0);
    }
}
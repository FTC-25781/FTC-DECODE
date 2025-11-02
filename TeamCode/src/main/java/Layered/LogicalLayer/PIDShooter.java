package Layered.LogicalLayer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;


public class PIDShooter {
    private DcMotorEx shooter;
    private Servo angleServo;
    private IMU imu;

    // PID Gains - tune these for your specific robot
    private final double kP = 0.0005;
    private final double kI = 0.00001;
    private final double kD = 0.0001;
    private final double kF = 0.3; // Feedforward gain

    private double targetRPM = 0;
    private double targetAngle = 0;
    private double integral = 0;
    private double outputPower = 0;
    private double previousRPM = 0;
    private double currentRPM = 0;

    // Kalman Filter for RPM estimation
    private KalmanFilter rpmKalman;
    private double filteredRPM = 0;

    private final ElapsedTime pidTimer = new ElapsedTime();
    private boolean isFirstUpdate = true;

    private double lastEncoderPosition = 0;

    private static final double TICKS_PER_REV = 51.9;
    private static final double MAX_MOTOR_RPM = 5000.0;
    private static final double INTEGRAL_MAX = 1000;
    private static final double INTEGRAL_MIN = -1000;
    private static final double MIN_DELTA_TIME = 0.005; // 5ms minimum
    private static final double MAX_ENCODER_JUMP = TICKS_PER_REV * 100; // Detect overflow

    private static final double RED_GOAL_HEIGHT = 40;
    private static final double BLUE_GOAL_HEIGHT = 40;
    private static final double SHOOTER_HEIGHT = 18;
    private static final double WHEEL_RADIUS = 2.0; // inches

    // Servo calibration - adjust these for your servo
    private static final double SERVO_MIN_ANGLE = 0.0;
    private static final double SERVO_MAX_ANGLE = 90.0;
    private static final double SERVO_MIN_POSITION = 0.0;
    private static final double SERVO_MAX_POSITION = 1.0;

    public PIDShooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        angleServo = hardwareMap.get(Servo.class, "angle_servo");

        try {
            imu = hardwareMap.get(IMU.class, "imu");
        } catch (Exception ignored) {
        }

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.COAST);

        // Initialize Kalman Filter for RPM
        // processNoise: how much we expect RPM to change naturally (motor dynamics)
        // measurementNoise: encoder noise and calculation errors
        rpmKalman = new KalmanFilter(0.0, 100.0, 50.0, 200.0);

        lastEncoderPosition = shooter.getCurrentPosition();
        pidTimer.reset();
    }

    public void setTargetRPM(double rpm) {
        double newTarget = Range.clip(rpm, 0, MAX_MOTOR_RPM);

        // Reset integral on large target changes to prevent windup
        if (Math.abs(newTarget - targetRPM) > 500) {
            integral = 0;
        }

        this.targetRPM = newTarget;
    }

    public boolean isAtTargetSpeed(double tolerance) {
        return targetRPM > 0 && Math.abs(targetRPM - filteredRPM) < tolerance;
    }

    public double getCurrentRPM() {
        return filteredRPM;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void update(Telemetry telemetry, double robotX, double robotY, double robotHeading, boolean isRedAlliance) {
        update(telemetry, robotX, robotY, 0, 0, robotHeading, isRedAlliance);
    }

    public void update(Telemetry telemetry, double robotX, double robotY, double robotVelX, double robotVelY, double robotHeading, boolean isRedAlliance) {
        double deltaTime = pidTimer.seconds();
        pidTimer.reset();

        // Skip first update or if deltaTime is too small
        if (isFirstUpdate || deltaTime < MIN_DELTA_TIME) {
            isFirstUpdate = false;
            lastEncoderPosition = shooter.getCurrentPosition();
            return;
        }

        // Calculate goal position and height
        double goalX = isRedAlliance ? ProjectileCalculations.RED_GOAL_X : ProjectileCalculations.BLUE_GOAL_X;
        double goalY = isRedAlliance ? ProjectileCalculations.RED_GOAL_Y : ProjectileCalculations.BLUE_GOAL_Y;
        double goalHeight = isRedAlliance ? RED_GOAL_HEIGHT : BLUE_GOAL_HEIGHT;

        // Get robot pitch from IMU if available
        double robotPitch = 0.0;
        if (imu != null) {
            try {
                robotPitch = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
            } catch (Exception e) {
                // IMU read failed, use 0
            }
        }

        // Transform robot velocity to shooter frame
        double angleToGoal = Math.atan2(goalY - robotY, goalX - robotX);
        double velTowardGoal = robotVelX * Math.cos(angleToGoal) + robotVelY * Math.sin(angleToGoal);

        // Calculate ballistics with movement compensation
        ProjectileCalculations.BallisticsResult ballistics =
                ProjectileCalculations.calculateBallisticsWithMovement(
                        robotX, robotY, goalX, goalY, goalHeight, SHOOTER_HEIGHT,
                        velTowardGoal, robotPitch
                );

        // Update target angle and RPM from ballistics
        targetAngle = ballistics.angle;
        targetRPM = ballistics.rpm;

        // Set servo position with proper calibration
        double servoPosition = mapAngleToServo(targetAngle);
        angleServo.setPosition(servoPosition);

        // Calculate current RPM from encoder
        double currentEncoderPosition = shooter.getCurrentPosition();
        double deltaPosition = currentEncoderPosition - lastEncoderPosition;

        // Detect encoder overflow or large jumps
        if (Math.abs(deltaPosition) > MAX_ENCODER_JUMP) {
            deltaPosition = 0;
        }

        // Calculate instantaneous (noisy) RPM measurement
        currentRPM = (deltaPosition / TICKS_PER_REV) * 60.0 / deltaTime;
        lastEncoderPosition = currentEncoderPosition;

        // Use Kalman Filter to estimate true RPM
        // Predict step: expects RPM to change based on applied power
        double predictedAcceleration = (outputPower - 0.1) * 10000.0; // Model motor dynamics
        rpmKalman.predict(deltaTime, predictedAcceleration);

        // Update step: incorporate noisy measurement
        rpmKalman.update(currentRPM);

        // Get filtered estimate
        filteredRPM = rpmKalman.getState();

        // PID Calculation using Kalman-filtered RPM
        double error = targetRPM - filteredRPM;

        // Integral with anti-windup
        integral += error * deltaTime;

        // Reset integral if target is zero
        if (targetRPM == 0) {
            integral = 0;
        }

        // Clamp integral
        integral = Range.clip(integral, INTEGRAL_MIN, INTEGRAL_MAX);

        // Derivative on measurement (not error) to avoid derivative kick
        double derivative = -(filteredRPM - previousRPM) / deltaTime;

        // Feedforward - scale by target RPM
        double feedforward = kF * targetRPM / MAX_MOTOR_RPM;

        // Calculate output power
        outputPower = feedforward + (kP * error) + (kI * integral) + (kD * derivative);
        outputPower = Range.clip(outputPower, 0.0, 1.0);

        // Apply power to motor
        if (targetRPM > 0) {
            shooter.setPower(outputPower);
        } else {
            shooter.setPower(0);
            integral = 0;
        }

        // Store RPM for next iteration's derivative
        previousRPM = filteredRPM;

        // Telemetry
        telemetry.addData("SH_Target RPM", "%.0f", targetRPM);
        telemetry.addData("SH_Filtered RPM", "%.0f", filteredRPM);
        telemetry.addData("SH_Raw RPM", "%.0f", currentRPM);
        telemetry.addData("SH_RPM Error", "%.0f", error);
        telemetry.addData("SH_Motor Power", "%.3f", outputPower);
        telemetry.addData("SH_Calculated Angle", "%.1f°", targetAngle);
        telemetry.addData("SH_Servo Pos", "%.2f", servoPosition);
        telemetry.addData("SH_Robot Pitch", "%.1f°", robotPitch);
        telemetry.addData("SH_Kalman Uncertainty", "%.1f", rpmKalman.getUncertainty());
    }

    private double mapAngleToServo(double angle) {
        double normalized = (angle - SERVO_MIN_ANGLE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);
        double servoPos = SERVO_MIN_POSITION + normalized * (SERVO_MAX_POSITION - SERVO_MIN_POSITION);
        return Range.clip(servoPos, SERVO_MIN_POSITION, SERVO_MAX_POSITION);
    }

    public void stop() {
        shooter.setPower(0);
        targetRPM = 0;
        integral = 0;
        rpmKalman.reset();
    }

    /**
     * Simple 1D Kalman Filter for RPM estimation
     * State: [RPM, RPM_rate]
     */
    private static class KalmanFilter {
        private double state;           // Current RPM estimate
        private double velocity;        // Rate of change of RPM
        private double uncertainty;     // Estimate uncertainty (P)
        private double velocityUncertainty;

        private final double processNoise;      // Q - how much RPM changes naturally
        private final double measurementNoise;  // R - encoder/calculation noise

        public KalmanFilter(double initialState, double initialUncertainty,
                            double processNoise, double measurementNoise) {
            this.state = initialState;
            this.velocity = 0;
            this.uncertainty = initialUncertainty;
            this.velocityUncertainty = initialUncertainty;
            this.processNoise = processNoise;
            this.measurementNoise = measurementNoise;
        }

        /**
         * Predict next state based on motor dynamics
         * @param dt Time step
         * @param acceleration Expected acceleration from motor power
         */
        public void predict(double dt, double acceleration) {
            // State prediction: RPM changes based on velocity and acceleration
            state = state + velocity * dt + 0.5 * acceleration * dt * dt;
            velocity = velocity + acceleration * dt;

            // Uncertainty grows with time (process noise)
            uncertainty += processNoise * dt;
            velocityUncertainty += processNoise * dt;
        }

        public void update(double measurement) {
            // Kalman Gain: how much to trust the measurement vs prediction
            double kalmanGain = uncertainty / (uncertainty + measurementNoise);

            // Update state estimate
            double innovation = measurement - state;
            state = state + kalmanGain * innovation;

            // Update velocity estimate (derivative of innovation)
            velocity = velocity + (kalmanGain * 0.1) * innovation;

            // Reduce uncertainty after incorporating measurement
            uncertainty = (1 - kalmanGain) * uncertainty;

            // Prevent uncertainty from becoming too small (avoid overconfidence)
            if (uncertainty < 1.0) uncertainty = 1.0;
        }

        public double getState() {
            return state;
        }

        public double getUncertainty() {
            return uncertainty;
        }

        public void reset() {
            state = 0;
            velocity = 0;
            uncertainty = 100.0;
        }
    }

    public static class ProjectileCalculations {

        private static final double RED_GOAL_X = 144;
        private static final double RED_GOAL_Y = 72;
        private static final double BLUE_GOAL_X = 0;
        private static final double BLUE_GOAL_Y = 72;
        private static final double GRAVITY = 386.4; // in/s^2
        private static final double WHEEL_RADIUS = 2.0; // inches
        private static final double MAX_MOTOR_RPM = 5000.0;

        // Energy loss compensation - tune this empirically
        private static final double ENERGY_LOSS_MULTIPLIER = 1.30;

        public static BallisticsResult calculateBallisticsWithMovement(
                double robotX, double robotY, double goalX, double goalY, double goalHeight,
                double shooterHeight, double robotVelTowardGoal, double robotPitch) {

            // Calculate distances
            double dx = goalX - robotX;
            double dy = goalY - robotY;
            double horizontalDist = Math.sqrt(dx * dx + dy * dy);
            double verticalDist = goalHeight - shooterHeight;

            // Choose launch angle based on distance
            double angle = chooseAngle(horizontalDist, verticalDist);

            // Compensate for robot pitch
            angle -= robotPitch;
            angle = Range.clip(angle, 25, 65);

            double angleRad = Math.toRadians(angle);

            // Calculate initial velocity needed for stationary robot
            double v0_stationary = calculateInitialVelocity(horizontalDist, verticalDist, angleRad);

            // Compensate for robot movement toward/away from goal
            double timeOfFlight = (2.0 * v0_stationary * Math.sin(angleRad)) / GRAVITY;
            double distanceTraveled = robotVelTowardGoal * timeOfFlight;

            // Recalculate for adjusted distance
            double adjustedHorizontalDist = horizontalDist - distanceTraveled;
            adjustedHorizontalDist = Math.max(adjustedHorizontalDist, 12.0);

            double v0_compensated = calculateInitialVelocity(adjustedHorizontalDist, verticalDist, angleRad);
            double v0 = Math.max(v0_compensated, 10.0);

            // Convert velocity to RPM
            double rpm = (v0 * 60.0) / (2.0 * Math.PI * WHEEL_RADIUS);
            rpm *= ENERGY_LOSS_MULTIPLIER;
            rpm = Range.clip(rpm, 800, MAX_MOTOR_RPM);

            return new BallisticsResult(rpm, angle);
        }
        private static double chooseAngle(double horizontalDist, double verticalDist) {
            double angle;

            if (horizontalDist < 60) {
                angle = 50;
            } else if (horizontalDist < 100) {
                angle = 45;
            } else {
                angle = 40;
            }

            if (verticalDist > 12) {
                angle += 5;
            } else if (verticalDist < -12) {
                angle -= 5;
            }

            return Range.clip(angle, 25, 65);
        }
        private static double calculateInitialVelocity(
                double horizontalDist, double verticalDist, double angleRad) {

            double cosTheta = Math.cos(angleRad);
            double tanTheta = Math.tan(angleRad);

            double numerator = GRAVITY * horizontalDist * horizontalDist;
            double denominator = 2.0 * cosTheta * cosTheta *
                    (horizontalDist * tanTheta - verticalDist);

            double velocitySquared;

            if (denominator <= 0) {
                velocitySquared = 500 * 500;
            } else {
                velocitySquared = numerator / denominator;
            }

            if (velocitySquared < 0) {
                velocitySquared = 10000;
            }

            return Math.sqrt(velocitySquared);
        }
        public static class BallisticsResult {
            public double rpm;
            public double angle;

            public BallisticsResult(double rpm, double angle) {
                this.rpm = rpm;
                this.angle = angle;
            }
        }
    }
}
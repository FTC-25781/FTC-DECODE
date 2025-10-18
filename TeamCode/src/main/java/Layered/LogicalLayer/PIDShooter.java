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


public class PIDShooter {
    private DcMotorEx shooter;
    private Servo angleServo;
    private IMU imu;

    // PID Gains
    private final double kP = 0.0005;
    private final double kI = 0.00001;
    private final double kD = 0.0001;
    private final double kF = 0.3; // Feedforward gain

    private double targetRPM = 0;
    private double targetAngle = 0;
    private double integral = 0;
    private double outputPower = 0;
    private double previousError = 0;
    private double currentRPM = 0; // Track current RPM for isAtTargetSpeed

    private final ElapsedTime pidTimer = new ElapsedTime();

    private double lastEncoderPosition = 0;

    private static final double TICKS_PER_REV = 51.9;
    private static final double MAX_MOTOR_RPM = 5000.0;
    private static final double INTEGRAL_MAX = 1000;
    private static final double INTEGRAL_MIN = -1000;

    private static final double RED_GOAL_HEIGHT = 40;
    private static final double BLUE_GOAL_HEIGHT = 40;
    private static final double SHOOTER_HEIGHT = 18;
    private static final double WHEEL_RADIUS = 2.0; // inches

    // Constructor: Initializes hardware and runs initial setup
    public PIDShooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        angleServo = hardwareMap.get(Servo.class, "angle_servo");

        try {
            imu = hardwareMap.get(IMU.class, "imu");
        } catch (Exception ignored) {
            // IMU not found, compensation disabled
        }

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        lastEncoderPosition = shooter.getCurrentPosition();
        pidTimer.reset();
    }

    public void setTargetRPM(double rpm) {
        this.targetRPM = Range.clip(rpm, 0, MAX_MOTOR_RPM);
    }

    public boolean isAtTargetSpeed(double tolerance) {
        return targetRPM > 0 && Math.abs(targetRPM - currentRPM) < tolerance;
    }

    public void update(Telemetry telemetry, double robotX, double robotY, double robotHeading, boolean isRedAlliance) {
        double deltaTime = pidTimer.seconds();
        pidTimer.reset();

        if (deltaTime == 0) {
            // Use 1ms as a fallback to avoid division by zero, though this is rare with ElapsedTime
            deltaTime = 0.001;
        }

        double goalX = isRedAlliance ? ProjectileCalculations.RED_GOAL_X : ProjectileCalculations.BLUE_GOAL_X;
        double goalY = isRedAlliance ? ProjectileCalculations.RED_GOAL_Y : ProjectileCalculations.BLUE_GOAL_Y;
        double goalHeight = isRedAlliance ? RED_GOAL_HEIGHT : BLUE_GOAL_HEIGHT;


        double robotVelX_shooter = 0; // Should come from Odometry

        ProjectileCalculations.BallisticsResult ballistics =
                ProjectileCalculations.calculateBallisticsWithMovement(
                        robotX, robotY, goalX, goalY, goalHeight, SHOOTER_HEIGHT,
                        robotVelX_shooter
                );

        // Update target angle (RPM is set externally by setTargetRPM for the PID)
        targetAngle = ballistics.angle;
        double servoPosition = Range.clip(targetAngle / 90.0, 0.0, 1.0);
        angleServo.setPosition(servoPosition);


        double currentEncoderPosition = shooter.getCurrentPosition();
        double deltaPosition = currentEncoderPosition - lastEncoderPosition;
        // (Ticks / TicksPerRev) * (60 seconds/minute) / DeltaTime
        currentRPM = (deltaPosition / TICKS_PER_REV) * 60.0 / deltaTime;
        lastEncoderPosition = currentEncoderPosition;

        // 3. PID Calculations
        double error = targetRPM - currentRPM;

        integral += error * deltaTime;
        // Reset integral if target is zero to prevent over-speeding on next target
        if (targetRPM == 0) integral = 0;
        integral = Range.clip(integral, INTEGRAL_MIN, INTEGRAL_MAX);

        double derivative = (error - previousError) / deltaTime;

        // Feedforward
        double feedforward = kF * targetRPM / MAX_MOTOR_RPM; // Scale kF by max RPM

        outputPower = feedforward + (kP * error) + (kI * integral) + (kD * derivative);
        outputPower = Range.clip(outputPower, 0.0, 1.0); // Shooter should only spin forward

        // 4. Apply Power
        if (targetRPM > 0) {
            shooter.setPower(outputPower);
        } else {
            shooter.setPower(0);
        }

        // Store error for next iteration
        previousError = error;

        // 5. Telemetry
        telemetry.addData("SH_Target RPM", "%.0f", targetRPM);
        telemetry.addData("SH_Current RPM", "%.0f", currentRPM);
        telemetry.addData("SH_RPM Error", "%.0f", error);
        telemetry.addData("SH_Motor Power", "%.3f", outputPower);
        telemetry.addData("SH_Calculated Angle", "%.1f°", targetAngle);
        telemetry.addData("SH_Servo Pos", "%.2f", servoPosition);
    }

    public static class ProjectileCalculations {

        private static final double RED_GOAL_X = 144;
        private static final double RED_GOAL_Y = 72;
        private static final double BLUE_GOAL_X = 0;
        private static final double BLUE_GOAL_Y = 72;
        private static final double GRAVITY = 386.4; // in/s^2
        private static final double WHEEL_RADIUS = 2.0; // inches
        private static final double MAX_MOTOR_RPM = 5000.0;

        public static BallisticsResult calculateBallisticsWithMovement(
                double robotX, double robotY, double goalX, double goalY, double goalHeight,
                double shooterHeight, double robotVelX_shooter) {

            // Calculate distances for stationary case
            double dx = goalX - robotX;
            double dy = goalY - robotY;
            double horizontalDist = Math.sqrt(dx * dx + dy * dy);
            double verticalDist = goalHeight - shooterHeight;

            double angle = chooseAngle(horizontalDist, verticalDist);
            double angleRad = Math.toRadians(angle);

            double v0_stationary = calculateInitialVelocity(horizontalDist, verticalDist, angleRad);

            // 3. Compensation for Moving Robot
            double v_comp_parallel = robotVelX_shooter / Math.cos(angleRad);
            double v0_compensated = v0_stationary + v_comp_parallel;
            double v0 = Math.max(v0_compensated, 10.0);

            // 4. Convert compensated velocity to RPM
            double rpm = (v0 * 60.0) / (2.0 * Math.PI * WHEEL_RADIUS);

            // Add compensation for energy losses
            rpm *= 1.30;

            // Clamp to motor capabilities
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
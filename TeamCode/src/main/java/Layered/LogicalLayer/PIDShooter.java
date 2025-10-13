package Layered.LogicalLayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="RPM Adjuster with PID", group="default")
public class PIDShooter extends LinearOpMode {
    private DcMotorEx shooter;
    private Servo angleServo;
    private IMU imu;

    private double kP = 0.0005;
    private double kI = 0.00001;
    private double kD = 0.0001;

    private double targetRPM = 0;
    private double targetAngle = 0;
    private double integral = 0;
    private double outputPower = 0;
    private double previousError = 0;

    private ElapsedTime pidTimer = new ElapsedTime();
    private ElapsedTime velocityTimer = new ElapsedTime();
    private static final double TICKS_PER_REV = 51.9;

    // Anti-windup limits
    private static final double INTEGRAL_MAX = 1000;
    private static final double INTEGRAL_MIN = -1000;

    // Field dimensions (inches)
    private static final double FIELD_WIDTH = 144;
    private static final double FIELD_LENGTH = 144;

    // Goal positions (inches)
    private static final double RED_GOAL_X = 144;
    private static final double RED_GOAL_Y = 72;
    private static final double RED_GOAL_HEIGHT = 40;

    private static final double BLUE_GOAL_X = 0;
    private static final double BLUE_GOAL_Y = 72;
    private static final double BLUE_GOAL_HEIGHT = 40;

    // Physics constants
    private static final double GRAVITY = 386.4; // in/s^2
    private static final double PROJECTILE_MASS = 75; // grams
    private static final double WHEEL_RADIUS = 2.0; // inches -

    // Robot state tracking
    private double robotX = 72; // inches - REPLACE WITH ODOMETRY, WILL NEED TO CHANGE
    private double robotY = 36; // inches - REPLACE WITH ODOMETRY, WILL NEED TO CHANGE
    private double previousX = 72;
    private double previousY = 36;
    private double robotVelocityX = 0; // inches/second
    private double robotVelocityY = 0; // inches/second
    private double robotHeading = 0; // radians
    private double shooterHeight = 18; // max height for robot

    // Alliance selection
    private boolean isRedAlliance = true;

    @Override
    public void runOpMode(){
        shooter = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        angleServo = hardwareMap.get(Servo.class, "angle_servo");

        // Initialize IMU
        try {
            imu = hardwareMap.get(IMU.class, "imu");
        } catch (Exception e) {
            telemetry.addData("IMU", "Not found - moving compensation disabled");
        }

        shooter.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addLine("Press X to toggle alliance");
        telemetry.addLine("Hold A to fire");
        telemetry.update();

        waitForStart();

        double lastEncoderPosition = shooter.getCurrentPosition();
        pidTimer.reset();
        velocityTimer.reset();

        while (opModeIsActive()){

            if (gamepad1.x) {
                isRedAlliance = !isRedAlliance;
                sleep(200); // Debounce
            }


            // Manual adjustment for testing only
            if (gamepad1.dpad_up) robotY += 1;
            if (gamepad1.dpad_down) robotY -= 1;
            if (gamepad1.dpad_right) robotX += 1;
            if (gamepad1.dpad_left) robotX -= 1;

            // robot velocity
            double velocityDeltaTime = velocityTimer.seconds();

            if (velocityDeltaTime > 0.05) { // Update every 50ms
                // Derivative of position = velocity
                robotVelocityX = (robotX - previousX) / velocityDeltaTime;
                robotVelocityY = (robotY - previousY) / velocityDeltaTime;

                previousX = robotX;
                previousY = robotY;
                velocityTimer.reset();
            }


            if (imu != null) {
                robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }

            // RPM and angle
            double goalX = isRedAlliance ? RED_GOAL_X : BLUE_GOAL_X;
            double goalY = isRedAlliance ? RED_GOAL_Y : BLUE_GOAL_Y;
            double goalHeight = isRedAlliance ? RED_GOAL_HEIGHT : BLUE_GOAL_HEIGHT;

            // Transform robot velocity to shooter reference frame
            double cosHeading = Math.cos(robotHeading);
            double sinHeading = Math.sin(robotHeading);
            double robotVelX_shooter = robotVelocityX * cosHeading + robotVelocityY * sinHeading;
            double robotVelY_shooter = -robotVelocityX * sinHeading + robotVelocityY * cosHeading;

            // Calculate required RPM and angle based on current position
            ProjectileCalculations.BallisticsResult ballistics =
                    ProjectileCalculations.calculateBallisticsWithMovement(
                            robotX, robotY, goalX, goalY, goalHeight, shooterHeight,
                            robotVelX_shooter, robotVelY_shooter
                    );

            targetRPM = ballistics.rpm;
            targetAngle = ballistics.angle;

            double servoPosition = Range.clip(targetAngle / 90.0, 0.0, 1.0);
            angleServo.setPosition(servoPosition);

           // A for shooting, release A if no shooting
            boolean shouldFire = gamepad1.a;
            if (!shouldFire) {
                targetRPM = 0;
                integral = 0; // Reset integral when not firing
            }

           // PID loop
            double deltaTime = pidTimer.seconds();
            pidTimer.reset();

            if (deltaTime == 0) {
                continue;
            }

            double currentEncoderPosition = shooter.getCurrentPosition();
            double deltaPosition = currentEncoderPosition - lastEncoderPosition;
            double currentRPM = (deltaPosition / TICKS_PER_REV) * 60.0 / deltaTime;
            lastEncoderPosition = currentEncoderPosition;

            // PID Calculations
            double error = targetRPM - currentRPM;

            integral += error * deltaTime;
            integral = Range.clip(integral, INTEGRAL_MIN, INTEGRAL_MAX);

            double derivative = (error - previousError) / deltaTime;

            outputPower = (kP * error) + (kI * integral) + (kD * derivative);

            outputPower = Range.clip(outputPower, -1.0, 1.0);

            shooter.setPower(outputPower);

            // Store error for next iteration
            previousError = error;

            double horizontalDistance = Math.sqrt(
                    Math.pow(goalX - robotX, 2) + Math.pow(goalY - robotY, 2)
            );
            double verticalDistance = goalHeight - shooterHeight;
            double robotSpeed = Math.sqrt(
                    robotVelocityX * robotVelocityX + robotVelocityY * robotVelocityY
            );
            double estimatedProjectileVelocity = (currentRPM * 2.0 * Math.PI * WHEEL_RADIUS) / 60.0;

            telemetry.addData("Status", shouldFire ? ">>> FIRING <<<" : "Ready to Fire");
            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addLine();

            telemetry.addData("Robot Position", "X: %.1f  Y: %.1f", robotX, robotY);
            telemetry.addData("Robot Speed", "%.1f in/s", robotSpeed);
            telemetry.addData("Robot Heading", "%.1f°", Math.toDegrees(robotHeading));
            telemetry.addLine();

            telemetry.addData("Distance to Goal", "%.1f inches", horizontalDistance);
            telemetry.addData("Height Difference", "%.1f inches", verticalDistance);
            telemetry.addLine();

            telemetry.addData("Calculated Angle", "%.1f°", targetAngle);
            telemetry.addData("Servo Position", "%.2f", servoPosition);
            telemetry.addLine();

            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("Current RPM", "%.0f", currentRPM);
            telemetry.addData("RPM Error", "%.0f", error);
            telemetry.addData("Motor Power", "%.3f", outputPower);
            telemetry.addLine();

            telemetry.addData("Projectile Velocity", "%.1f in/s", estimatedProjectileVelocity);
            telemetry.addLine();

            telemetry.addData("Controls", "A: Fire");
            telemetry.addData("", "X: Toggle Alliance");
            telemetry.addData("", "D-Pad: Adjust Position (Test)");
            telemetry.update();
        }

        shooter.setPower(0);
    }

    public static class ProjectileCalculations {

        private static final double GRAVITY = 386.4; // in/s^2
        private static final double WHEEL_RADIUS = 2.0; // inches

        public static BallisticsResult calculateBallisticsWithMovement(
                double robotX, double robotY, double goalX, double goalY, double goalHeight,
                double shooterHeight, double robotVelX_shooter, double robotVelY_shooter) {

            // Calculate initial distance from robot to goal
            double dx = goalX - robotX;
            double dy = goalY - robotY;
            double horizontalDist = Math.sqrt(dx * dx + dy * dy);
            double verticalDist = goalHeight - shooterHeight;

            //optimal launch angle
            double angle = chooseAngle(horizontalDist, verticalDist);
            double angleRad = Math.toRadians(angle);

            //for stationary robot
            double v0 = calculateInitialVelocity(horizontalDist, verticalDist, angleRad);

            //for moving robot
            for (int iteration = 0; iteration < 3; iteration++) {
                // Calculate projectile velocity components in shooter frame
                double vProjectileX = v0 * Math.cos(angleRad);
                double vProjectileY = v0 * Math.sin(angleRad);

                // Combined velocity (projectile velocity + robot velocity)
                double vTotalX = vProjectileX + robotVelX_shooter;
                double vTotalY = vProjectileY + robotVelY_shooter;

                // Estimate time of flight: t = distance / horizontal_velocity
                double estimatedFlightTime = horizontalDist / Math.max(vTotalX, 1.0);

                // Predict future robot position when projectile lands
                // Position = current + velocity * time
                double futureRobotX = robotX + (robotVelX_shooter * estimatedFlightTime);
                double futureRobotY = robotY + (robotVelY_shooter * estimatedFlightTime);

                // Recalculate distance from predicted future position
                dx = goalX - futureRobotX;
                dy = goalY - futureRobotY;
                horizontalDist = Math.sqrt(dx * dx + dy * dy);

                // Recalculate required velocity for new distance
                v0 = calculateInitialVelocity(horizontalDist, verticalDist, angleRad);

                // Adjust for robot velocity component
                // The shooter needs to provide: v_total - v_robot
                v0 = Math.max(v0 - robotVelX_shooter, 0);
            }

            // Convert velocity to RPM

            double rpm = (v0 * 60.0) / (2.0 * Math.PI * WHEEL_RADIUS);

            // Add compensation for energy losses
            // 30% increase accounts for:
            // - Friction during projectile launch (15%)
            // - Air resistance (5%)
            // - Spin/energy transfer efficiency (10%)
            rpm *= 1.30;

            // Clamp to motor capabilities
            rpm = Range.clip(rpm, 800, 5000);

            return new BallisticsResult(rpm, angle);
        }

        /**
         * Choose launch angle based on distance to target
         * Closer shots use steeper angles, farther shots use flatter angles
         */
        private static double chooseAngle(double horizontalDist, double verticalDist) {
            double angle;

            if (horizontalDist < 60) {
                angle = 50; // Steeper for close shots (< 5 feet)
            } else if (horizontalDist < 100) {
                angle = 45; // Standard 45° for medium range
            } else {
                angle = 40; // Flatter for long shots (> 8 feet)
            }

            // Adjust for height difference
            if (verticalDist > 12) {
                angle += 5; // Increase angle if goal is significantly higher
            } else if (verticalDist < 0) {
                angle -= 5; // Decrease angle if goal is lower
            }

            // Clamp to mechanically feasible range
            return Range.clip(angle, 25, 65);
        }

        private static double calculateInitialVelocity(
                double horizontalDist, double verticalDist, double angleRad) {

            double cosTheta = Math.cos(angleRad);
            double sinTheta = Math.sin(angleRad);
            double tanTheta = Math.tan(angleRad);

            // Apply kinematic equation
            double numerator = GRAVITY * horizontalDist * horizontalDist;
            double denominator = 2.0 * cosTheta * cosTheta *
                    (horizontalDist * tanTheta - verticalDist);

            double velocitySquared;

            // Check if trajectory is possible at this angle
            if (denominator <= 0) {
                // Use fallback: vertical kinematic equation
                double peakHeight = Math.max(verticalDist + 12, 24);
                velocitySquared = (2.0 * GRAVITY * peakHeight) / (sinTheta * sinTheta);
            } else {
                velocitySquared = numerator / denominator;
            }

            // Safety check
            if (velocitySquared < 0) {
                velocitySquared = 10000; // Minimum safe velocity (100 in/s)
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
package LogicalLayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="RPM Adjuster with PID", group="default")
public class PIDShooter extends LinearOpMode {
    private DcMotorEx shooter;
    private double kP = 0.0005;
    private double kI = 0.00001;
    private double kD = 0.0001;
    private double targetRPM = 0;
    private double integral = 0;
    private double outputPower = 0;
    private double previousError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();
    private static final double TICKS_PER_REV = 51.9;

    @Override
    public void runOpMode(){
        shooter = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooter.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        waitForStart();

        double lastEncoderPosition = shooter.getCurrentPosition();
        pidTimer.reset();

        while (opModeIsActive()){
            if (gamepad1.a) {
                targetRPM = 2500; // Low speed
            } else if (gamepad1.b) {
                targetRPM = 3000; // High speed
            } else if (gamepad1.x) {
                targetRPM = 0; // Stop motor
            }

            double currentTime = pidTimer.seconds();
            double currentEncoderPosition = shooter.getCurrentPosition();

            double deltaTime = currentTime;
            double deltaPosition = currentEncoderPosition - lastEncoderPosition;

            double currentRPM = (deltaPosition / TICKS_PER_REV ) * 60 / deltaTime;

            lastEncoderPosition = currentEncoderPosition;
            pidTimer.reset();

            double error = targetRPM - currentRPM;
            integral += error * deltaTime;
            double derivative = (error - previousError) / deltaTime;
            outputPower = Range.clip(outputPower, -1.0, 1.0);

            shooter.setPower(outputPower);

            previousError = error;
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM", currentRPM);
            telemetry.addData("Motor Power", outputPower);
            telemetry.addData("Error", error);
            telemetry.update();
        }
    }
}



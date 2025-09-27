package Layered.LogicalLayer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import Layered.PhysicalLayer.Motor;

@TeleOp(name="RPM Adjuster with PID", group="default")
public class PIDShooter extends LinearOpMode {
    private Motor shooter;  // Use your wrapper class
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
        // Wrap the hardware motor with your Motor class
        DcMotorEx rawMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        shooter = new Motor(rawMotor, battery);


        shooter.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        double lastEncoderPosition = shooter.getCurrentPosition();
        pidTimer.reset();

        while (opModeIsActive()){
            if (gamepad1.a) {
                targetRPM = 2500;
            } else if (gamepad1.b) {
                targetRPM = 3000;
            } else if (gamepad1.x) {
                targetRPM = 0;
            }

            double currentTime = pidTimer.seconds();
            double currentEncoderPosition = shooter.getCurrentPosition();

            double deltaTime = currentTime;
            double deltaPosition = currentEncoderPosition - lastEncoderPosition;

            double currentRPM = (deltaPosition / TICKS_PER_REV) * 60 / deltaTime;

            lastEncoderPosition = currentEncoderPosition;
            pidTimer.reset();

            double error = targetRPM - currentRPM;
            integral += error * deltaTime;
            double derivative = (error - previousError) / deltaTime;

            // *** Compute PID output ***
            outputPower = (kP * error) + (kI * integral) + (kD * derivative);
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

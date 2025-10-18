package Layered.ControlLayer;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import Layered.LogicalLayer.PIDShooter;
// import Layered.ControlLayer.Intake.INTAKE_STATE; // Removed unused import statement

public class Deposit {

    private PIDShooter shooter;
    private Intake intake;
    private Servo pusherServo; // Mechanism to push the ball into the flywheel

    public enum OUTTAKE_STATE {
        IDLE,
        ACCELERATING,
        TRANSFER_BALL,
        FIRING,
        RELOAD
    }

    private OUTTAKE_STATE currentState = OUTTAKE_STATE.IDLE;

    // --- Tuning Constants ---
    private final double FIRE_RPM_TOLERANCE = 50.0; // Tolerance for PIDShooter speed check
    private final double PUSH_DELAY_SECONDS = 0.5; // Time to hold the pusher forward
    private final double RETRACT_DELAY_SECONDS = 0.2; // Time to allow pusher to retract
    private final double INTAKE_TRANSFER_DELAY = 0.1; // Time for intake to reverse slightly
    private final double PUSHER_RETRACT = 0.2; // Servo position for pusher home
    private final double PUSHER_EXTEND = 0.8; // Servo position for pusher firing

    private ElapsedTime stateTimer = new ElapsedTime();

    public Deposit(HardwareMap hardwareMap, PIDShooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;

        pusherServo = hardwareMap.get(Servo.class, "pusherServo");
        pusherServo.setPosition(PUSHER_RETRACT);
    }

    public void update(Telemetry telemetry, boolean fireButton, double targetRPM) {

        // PIDShooter is updated internally via the Outtake. The Outtake controls its target speed.
        shooter.setTargetRPM(targetRPM);
        shooter.update(telemetry, 0, 0, 0, false); // Update the PID loop

        switch (currentState) {
            case IDLE:
                pusherServo.setPosition(PUSHER_RETRACT);

                // If Fire button pressed AND we have a ball, start accelerating
                if (fireButton && intake.getBallCount() > 0) {
                    currentState = OUTTAKE_STATE.ACCELERATING;
                    stateTimer.reset();
                }
                break;

            case ACCELERATING:
                telemetry.addData("OT_Status", "ACCELERATING...");

                // Check if the shooter is at the required speed
                // NOTE: The compiler expects isAtTargetSpeed() to be a method in PIDShooter.
                if (shooter.isAtTargetSpeed(FIRE_RPM_TOLERANCE)) {
                    currentState = OUTTAKE_STATE.TRANSFER_BALL;
                    intake.startOuttakeTransfer(); // Tell intake to move ball into firing position (REVERSING)
                    stateTimer.reset();
                }
                break;

            case TRANSFER_BALL:
                telemetry.addData("OT_Status", "TRANSFERRING BALL");
                // Wait briefly for the Intake to push the ball slightly forward/stabilize it
                if (stateTimer.seconds() >= INTAKE_TRANSFER_DELAY) {
                    currentState = OUTTAKE_STATE.FIRING;
                    stateTimer.reset();
                }
                break;

            case FIRING:
                // Extend the pusher
                pusherServo.setPosition(PUSHER_EXTEND);
                telemetry.addData("OT_Status", "FIRING");

                // Wait for push to complete
                if (stateTimer.seconds() >= PUSH_DELAY_SECONDS) {
                    currentState = OUTTAKE_STATE.RELOAD;
                    // IMPORTANT: Decrement the ball count the moment the ball is pushed out
                    intake.decrementBallCount();
                    stateTimer.reset();
                }
                break;

            case RELOAD:
                // Retract the pusher
                pusherServo.setPosition(PUSHER_RETRACT);
                telemetry.addData("OT_Status", "RELOADING");

                // Wait for pusher to retract fully
                if (stateTimer.seconds() >= RETRACT_DELAY_SECONDS) {
                    currentState = OUTTAKE_STATE.IDLE;
                }
                break;
        }

        telemetry.addData("OT_State", currentState);
        telemetry.addData("OT_Pusher Pos", "%.2f", pusherServo.getPosition());
    }
}
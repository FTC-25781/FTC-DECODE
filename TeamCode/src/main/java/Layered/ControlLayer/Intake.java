package Layered.ControlLayer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.SRSHub;

@TeleOp(name="Intake Test", group="Sensor")
public class Intake extends OpMode {

    private DcMotorEx intakeMotor;
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
    private final double INTAKE_POWER = 0.8;
    private final double REVERSE_POWER = -0.5;
    private final int MAX_BALLS = 3;

    @Override
    public void init() {
        // Hardware Initialization
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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
            if (distances != null && distances.length > 0) {
                double sum = 0;
                int validCount = 0;
                for (short d : distances) {
                    if (d > 0) {
                        sum += d;
                        validCount++;
                    }
                }
                if (validCount > 0) {
                    avgDistanceInches = (sum / validCount) / 25.4;
                }
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
                intakeMotor.setPower(0);
                break;

            case INTAKING:
                intakeMotor.setPower(INTAKE_POWER);
                break;

            case REVERSING:
                intakeMotor.setPower(REVERSE_POWER);
                break;
        }

        lastState = currentState;

        telemetry.addData("IN_State", currentState);
        telemetry.addData("IN_Ball Count", ballCount + "/" + MAX_BALLS);
        telemetry.addData("IN_Distance", "%.2f inches", avgDistanceInches);
        telemetry.addData("IN_Motor Power", "%.0f%%", intakeMotor.getPower() * 100);
        telemetry.update();
    }

    @Override
    public void stop() {
        intakeMotor.setPower(0);
    }
}
package Layered.ControlLayer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HubTest;
import org.firstinspires.ftc.teamcode.SRSHub;

@TeleOp(name="Intake System")
public class Intake extends OpMode{

    private DcMotorEx intakeMotor;
    private SRSHub.VL53L5CX tofSensor;
    private SRSHub srsHub;

    private enum INTAKE_STATE {
        IDLE,
        INTAKING,
        FULL
    }

    private INTAKE_STATE currentState = INTAKE_STATE.IDLE;

    private int ballCount = 0;
    private boolean lastSensorBlocked = false; // true = detecting something

    private final double BALL_THRESHOLD_IN = 5.0;

    @Override
    public void init(){
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        SRSHub.Config config = new SRSHub.Config();

        SRSHub.VL53L5CX tofSensor = new SRSHub.VL53L5CX(SRSHub.VL53L5CX.Resolution.GRID_4x4);
        config.addI2CDevice(
                2, tofSensor);

        RobotLog.clearGlobalWarningMsg();

        SRSHub hub = hardwareMap.get(
                SRSHub.class,
                "srs"
        );

        hub.init(config);
        telemetry.addLine("Intake System Initialized");
        telemetry.update();
    }

    @Override
    public void loop(){
        srsHub.update();

        double avgDistanceInches = 999; // default reading, since 999 > 5.0, no ball detected
        boolean currentSensorBlocked = false;

        if (srsHub.disconnected()) {
            telemetry.addLine("srshub disconnected");
        }
        else{
            if(!tofSensor.disconnected){
                short[] distances = tofSensor.distances;
                if(distances.length > 0){
                    double average = 0;
                    double count = 0;

                    for(short i : distances) {
                        average += i;
                        count++;
                    }
                    avgDistanceInches = (average/count) / 25.4;
            }
            else{
                telemetry.addLine("TOF sensor disconnected");
                telemetry.update();
            }
        }
    }
        currentSensorBlocked = avgDistanceInches <= BALL_THRESHOLD_IN;
        /* This is the same logic as the statement above
        if(avgDistanceInches<=BALL_THRESHOLD_IN){
            currentSensorBlocked = true;
        }
        else{
            currentSensorBlocked = false;
        }
         */

        if(currentSensorBlocked && !lastSensorBlocked){
            ballCount++;
            currentState = INTAKE_STATE.INTAKING;
        }

        lastSensorBlocked = currentSensorBlocked;

        if(ballCount >= 3 && currentState == INTAKE_STATE.INTAKING){
        currentState = INTAKE_STATE.FULL;
        }

        switch (currentState){
            case IDLE:
                intakeMotor.setPower(0);
                break;

            case INTAKING:
                gamepad1.rumble(1000);
                telemetry.addLine("Press A to starting intake motor");
                if(gamepad1.a){
                    intakeMotor.setPower(1.0);
                }

            case FULL:
                intakeMotor.setPower(0);
                telemetry.addLine("Intake Full (3 Balls)");
                break;
        }
        telemetry.addData("State", currentState);
        telemetry.addData("Sensor", currentSensorBlocked ? "Clear" : "Ball Found");// Clear if true (object detected), blocked if false (object not detected)
        telemetry.addData("Ball Count", ballCount);
        telemetry.update();
    }
}

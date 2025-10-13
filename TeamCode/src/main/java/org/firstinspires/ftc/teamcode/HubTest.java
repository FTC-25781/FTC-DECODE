package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name = "HubTest")
public class HubTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        // All ports default to NONE, buses default to empty
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

        while (!hub.ready());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            hub.update();

            if (hub.disconnected()) {
                multipleTelemetry.addLine("srshub disconnected");
            }
            else{
                if(!tofSensor.disconnected){
                    short[] distances = tofSensor.distances;
                    float[] distancesInMeters = new float[distances.length];

                    for(int i = 0; i < distances.length; i++){
                        distancesInMeters[i] = distances[i] / 1000.0f; // mm to m
                    }
                }
                else{
                    telemetry.addLine("TOF sensor disconnected");
                    telemetry.update();
                }
            }
            multipleTelemetry.update();
        }
    }
}

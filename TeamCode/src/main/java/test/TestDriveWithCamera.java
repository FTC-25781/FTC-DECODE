package test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import android.content.Context;

import Layered.PhysicalLayer.Camera;

@TeleOp(name = "TestDriveWithCamera", group = "Test")
public class TestDriveWithCamera extends OpMode {

    // Drive motors
    private DcMotor leftFront, leftRear, rightFront, rightRear;

    // Camera
    private Camera camera;

    @Override
    public void init() {
        // Map drive motors (adjust names to match your config)
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // Reverse right side so forward stick = forward
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Initialize camera
        Context appContext = hardwareMap.appContext;
        camera = new Camera(hardwareMap, appContext);

        telemetry.addLine("Initialized. Ready to start!");
    }

    @Override
    public void loop() {
        // === DRIVING CONTROL (arcade style) ===
        double drive = -gamepad1.left_stick_y;  // Forward/back
        double turn = gamepad1.right_stick_x;   // Turning

        double leftPower = drive + turn;
        double rightPower = drive - turn;

        leftFront.setPower(leftPower);
        leftRear.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightRear.setPower(rightPower);

        // === CAMERA PROCESSING ===
        camera.processFrame();

        // === TELEMETRY OUTPUT ===
        telemetry.addLine("=== Camera ===");
        telemetry.addData("Status", camera.getCameraStatus());
        telemetry.addData("Last Tag ID", camera.getLastDetectedTagId());
        telemetry.addData("Last Detection Time", camera.getLastDetectionTime());
        telemetry.addData("Database Count", camera.getDetectionCount());

        telemetry.update();
    }

    @Override
    public void stop() {
        // Shutdown camera + database
        if (camera != null) {
            camera.close();
        }
    }
}

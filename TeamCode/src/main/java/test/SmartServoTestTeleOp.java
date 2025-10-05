package test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import Layered.PhysicalLayer.SmartServo; // Your custom servo class
import Layered.PhysicalLayer.SmartServo.ServoType;
import Layered.PhysicalLayer.SmartServo.ServoState;

@TeleOp(name = "Smart Servo Test", group = "Testing")
public class SmartServoTestTeleOp extends OpMode {

    private SmartServo testServo;
    private double angleIncrement = 5; // degrees per step
    private double chamberOffset = 0.0;
    private ElapsedTime debounce = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addLine("Initializing Smart Servo...");
        try {
            testServo = new SmartServo(hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "testServo"), ServoType.STANDARD_180);
        } catch (Exception e) {
            telemetry.addData("ERROR", "Servo not found! Check configuration name: testServo");
            return;
        }

        testServo.calibrate();
        telemetry.addLine("Smart Servo ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
        Gamepad g1 = gamepad1;

        // === ANGLE ADJUSTMENT ===
        if (g1.dpad_up && debounce.seconds() > 0.15) {
            testServo.adjustAngleByDegrees(angleIncrement);
            debounce.reset();
        }
        if (g1.dpad_down && debounce.seconds() > 0.15) {
            testServo.adjustAngleByDegrees(-angleIncrement);
            debounce.reset();
        }

        // === CHAMBER ADJUSTMENT ===
        if (g1.left_bumper && debounce.seconds() > 0.15) {
            chamberOffset -= 0.02;
            testServo.adjustChamber(chamberOffset);
            debounce.reset();
        }
        if (g1.right_bumper && debounce.seconds() > 0.15) {
            chamberOffset += 0.02;
            testServo.adjustChamber(chamberOffset);
            debounce.reset();
        }

        // === CALIBRATE (press A) ===
        if (g1.a && debounce.seconds() > 1.0) {
            testServo.calibrate();
            debounce.reset();
        }

        // === SET TO 90° (press X) ===
        if (g1.x && debounce.seconds() > 0.5) {
            testServo.setAngleDegrees(90);
            debounce.reset();
        }

        // === RESET POSITION (press B) ===
        if (g1.b && debounce.seconds() > 0.5) {
            testServo.setAngleDegrees(0);
            debounce.reset();
        }

        // === TELEMETRY OUTPUT ===
        telemetry.addLine("=== Smart Servo Telemetry ===");
        telemetry.addData("State", testServo.getState());
        telemetry.addData("Current Angle (deg)", "%.2f", testServo.getCurrentAngleDegrees());
        telemetry.addData("Servo Position", "%.3f", testServo.getPosition());
        telemetry.addData("Target Position", "%.3f", testServo.isAtTarget() ? "Reached" : "Moving");
        telemetry.addData("Servo Type", testServo.getServoType());
        telemetry.addData("Chamber Offset", "%.3f", chamberOffset);
        telemetry.addData("Direction", testServo.getDirection());
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("Dpad ↑↓: Adjust angle ±5°");
        telemetry.addLine("Bumpers: Adjust chamber offset");
        telemetry.addLine("A: Calibrate  |  X: Go to 90°  |  B: Go to 0°");
        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode.layered.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.bylazar.telemetry.PanelsTelemetry;

@Configurable
@TeleOp(name = "PID Controlled CRServo", group = "tests")
public class MotEncoderPlusIntakeServo extends LinearOpMode {

    // --- PID TUNING CONSTANTS ---
    // These values MUST be carefully tuned on your specific robot setup.
    // Start with kP only, then kD, then kI.
    private static final double kP = 0.0005; // Proportional Gain: Adjusts power based on current error.
    private static final double kI = 0.0;    // Integral Gain: Eliminates steady-state error (start at 0).
    private static final double kD = 0.001; // Derivative Gain: Damps oscillation (use small values).

    // Anti-windup constant: Limits the maximum contribution of the integral term.
    private static final double INTEGRAL_MAX_POWER = 0.2;

    private DcMotorEx encoder;
    private CRServo servo;

    // Target state variables
    private int targetPosition = 400; // Default target in encoder ticks
    private final int POSITION_A = 400;
    private final int POSITION_B = 800;

    // PID state variables
    private double lastError = 0;
    private double integralSum = 0;
    private long lastTime = 0;


    /**
     * Calculates the PID output power based on the current state and reference.
     * @param reference The target position (encoder ticks).
     * @param state The current encoder position (encoder ticks).
     * @return The calculated output power (velocity) for the CR Servo [-1.0 to 1.0].
     */
    private double calculatePID(int reference, int state) {
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastTime) / 1e9; // Convert nanoseconds to seconds
        lastTime = currentTime;

        // 1. Calculate Error
        double error = reference - state;

        // 2. Proportional Term (P)
        double pTerm = kP * error;

        // 3. Integral Term (I) - Accumulates error over time (only when near the target)
        if (Math.abs(error) < 50) { // Integrate only when close to prevent large integral windup
            integralSum += error * deltaTime;
        } else {
            // Reset integral sum if far from the target to avoid windup
            integralSum = 0;
        }

        // Apply integral windup prevention (limit the integral sum's maximum contribution)
        // Max integral contribution = INTEGRAL_MAX_POWER. So, integralSum * kI <= INTEGRAL_MAX_POWER
        double maxIntegral = INTEGRAL_MAX_POWER / kI;
        integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);

        double iTerm = kI * integralSum;

        // 4. Derivative Term (D) - Reacts to the rate of change of error
        double dTerm = kD * (error - lastError) / deltaTime;
        lastError = error;

        // 5. Calculate Total Output
        double output = pTerm + iTerm + dTerm;

        // Clamp output to the valid power range for CR Servos [-1.0, 1.0]
        return Range.clip(output, -1.0, 1.0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Hardware Initialization ---
        encoder = hardwareMap.get(DcMotorEx.class, "encoder");
        servo = hardwareMap.get(CRServo.class, "transfer1");

        // Set directions as in original code
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset encoder position to 0
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // Run mode for reading only

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Target (A)", POSITION_A);
        telemetry.addData("Target (B)", POSITION_B);
        telemetry.update();

        waitForStart();

        lastTime = System.nanoTime(); // Initialize time for derivative calculation

        while (opModeIsActive()) {
            // --- User Input & Target Adjustment ---
            if (gamepad1.a) {
                targetPosition = POSITION_A;
                integralSum = 0; // Reset integral sum on new target
            } else if (gamepad1.b) {
                targetPosition = POSITION_B;
                integralSum = 0; // Reset integral sum on new target
            } else if (gamepad1.x) {
                // Emergency Stop/Zeroing
                targetPosition = 0;
                integralSum = 0;
                encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            }


            // --- PID Control Loop ---
            int currentPosition = encoder.getCurrentPosition();
            double power = calculatePID(targetPosition, currentPosition);

            // Set the CR Servo power
            servo.setPower(power);


            // --- Telemetry Output ---
            telemetry.addData("Status", "Running");
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Power Output", String.format("%.3f", power));
            telemetry.addData("Error", targetPosition - currentPosition);
            telemetry.addData("Integral Sum", String.format("%.3f", integralSum));
            telemetry.update();
        }

        // Stop servo once OpMode ends
        servo.setPower(0);
    }
}

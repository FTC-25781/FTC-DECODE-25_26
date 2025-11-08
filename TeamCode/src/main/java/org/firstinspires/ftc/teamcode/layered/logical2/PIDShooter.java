package org.firstinspires.ftc.teamcode.layered.logical2;

import org.firstinspires.ftc.teamcode.layered.physical1.Motor;
import Layered.PhysicalLayer.SmartServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

public class PIDShooter {
    // Using Physical Layer wrappers
    private Motor shooter;
    private SmartServo angleServo;
    private IMU imu;

    // PID Gains
    private final double kP = 0.0005;
    private final double kI = 0.00001;
    private final double kD = 0.0001;
    private final double kF = 0.3;

    private double targetRPM = 0;
    private double targetAngle = 0;
    private double integral = 0;
    private double outputPower = 0;
    private double previousRPM = 0;
    private double filteredRPM = 0;

    private KalmanFilter rpmKalman;
    private final ElapsedTime pidTimer = new ElapsedTime();
    private boolean isFirstUpdate = true;

    private static final double MAX_MOTOR_RPM = 5000.0;
    private static final double INTEGRAL_MAX = 1000;
    private static final double INTEGRAL_MIN = -1000;
    private static final double MIN_DELTA_TIME = 0.005;

    // ✅ CORRECTED GOAL HEIGHTS - Both goals are 47 inches high
    private static final double GOAL_HEIGHT = 47.0;
    private static final double SHOOTER_HEIGHT = 18.0;

    // Constructor now accepts Physical Layer objects
    public PIDShooter(Motor shooterMotor, SmartServo angleServo, IMU imu) {
        this.shooter = shooterMotor;
        this.angleServo = angleServo;
        this.imu = imu;

        // Use Physical layer methods
        shooter.setVoltageCap(12); // Use voltage capping feature

        // Initialize Kalman Filter
        rpmKalman = new KalmanFilter(0.0, 100.0, 50.0, 200.0);
        pidTimer.reset();
    }

    public void setTargetRPM(double rpm) {
        double newTarget = Range.clip(rpm, 0, MAX_MOTOR_RPM);

        if (Math.abs(newTarget - targetRPM) > 500) {
            integral = 0;
        }

        this.targetRPM = newTarget;
    }

    public boolean isAtTargetSpeed(double tolerance) {
        return targetRPM > 0 && Math.abs(targetRPM - filteredRPM) < tolerance;
    }

    public double getCurrentRPM() {
        return filteredRPM;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Main update method - call this from your TeleOp/Auto loop
     *
     * @param telemetry FTC telemetry for debugging
     * @param robotX Current robot X position from Pinpoint (inches)
     * @param robotY Current robot Y position from Pinpoint (inches)
     * @param robotHeading Current robot heading from Pinpoint (degrees)
     * @param isRedAlliance true if on red alliance, false if blue
     */
    public void update(Telemetry telemetry, double robotX, double robotY,
                       double robotHeading, boolean isRedAlliance) {
        update(telemetry, robotX, robotY, 0, 0, robotHeading, isRedAlliance);
    }

    /**
     * Advanced update with velocity compensation
     *
     * @param robotVelX Robot velocity in X direction (inches/sec)
     * @param robotVelY Robot velocity in Y direction (inches/sec)
     */
    public void update(Telemetry telemetry, double robotX, double robotY,
                       double robotVelX, double robotVelY, double robotHeading,
                       boolean isRedAlliance) {
        double deltaTime = pidTimer.seconds();
        pidTimer.reset();

        if (isFirstUpdate || deltaTime < MIN_DELTA_TIME) {
            isFirstUpdate = false;
            return;
        }

        // ✅ Get correct goal coordinates based on alliance
        double goalX = isRedAlliance ? ProjectileCalculations.RED_GOAL_X : ProjectileCalculations.BLUE_GOAL_X;
        double goalY = isRedAlliance ? ProjectileCalculations.RED_GOAL_Y : ProjectileCalculations.BLUE_GOAL_Y;

        // Get robot pitch from IMU if available
        double robotPitch = 0.0;
        if (imu != null) {
            try {
                robotPitch = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
            } catch (Exception e) {
                // IMU read failed, use 0
            }
        }

        // Transform robot velocity to shooter frame
        double angleToGoal = Math.atan2(goalY - robotY, goalX - robotX);
        double velTowardGoal = robotVelX * Math.cos(angleToGoal) + robotVelY * Math.sin(angleToGoal);

        // ✅ Calculate ballistics with correct goal position and height
        ProjectileCalculations.BallisticsResult ballistics =
                ProjectileCalculations.calculateBallisticsWithMovement(
                        robotX, robotY, goalX, goalY, GOAL_HEIGHT, SHOOTER_HEIGHT,
                        velTowardGoal, robotPitch
                );

        targetAngle = ballistics.angle;
        targetRPM = ballistics.rpm;

        // ✅ Use SmartServo's angle-based control (Physical layer feature)
        angleServo.setAngleDegrees(targetAngle);

        // ✅ Use Motor's built-in RPM measurement (Physical layer feature)
        double currentRPM = shooter.getRPM();

        // Kalman Filter prediction
        double predictedAcceleration = (outputPower - 0.1) * 10000.0;
        rpmKalman.predict(deltaTime, predictedAcceleration);
        rpmKalman.update(currentRPM);
        filteredRPM = rpmKalman.getState();

        // PID Calculation
        double error = targetRPM - filteredRPM;

        integral += error * deltaTime;
        if (targetRPM == 0) {
            integral = 0;
        }
        integral = Range.clip(integral, INTEGRAL_MIN, INTEGRAL_MAX);

        double derivative = -(filteredRPM - previousRPM) / deltaTime;
        double feedforward = kF * targetRPM / MAX_MOTOR_RPM;

        outputPower = feedforward + (kP * error) + (kI * integral) + (kD * derivative);
        outputPower = Range.clip(outputPower, 0.0, 1.0);

        // ✅ Use Physical layer setPower (includes voltage compensation)
        if (targetRPM > 0) {
            shooter.setPower(outputPower);
        } else {
            shooter.setPower(0);
            integral = 0;
        }

        previousRPM = filteredRPM;

        // ✅ Enhanced Telemetry with distance info
        double distanceToGoal = Math.sqrt(Math.pow(goalX - robotX, 2) + Math.pow(goalY - robotY, 2));

        telemetry.addData("SH_Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("SH_Distance to Goal", "%.1f in", distanceToGoal);
        telemetry.addData("SH_Robot Position", "(%.1f, %.1f)", robotX, robotY);
        telemetry.addData("SH_Goal Position", "(%.1f, %.1f)", goalX, goalY);
        telemetry.addData("SH_Target RPM", "%.0f", targetRPM);
        telemetry.addData("SH_Filtered RPM", "%.0f", filteredRPM);
        telemetry.addData("SH_Raw RPM", "%.0f", currentRPM);
        telemetry.addData("SH_RPM Error", "%.0f", error);
        telemetry.addData("SH_Motor Power", "%.3f", outputPower);
        telemetry.addData("SH_Calculated Angle", "%.1f°", targetAngle);
        telemetry.addData("SH_Servo Angle", "%.1f°", angleServo.getCurrentAngleDegrees());
        telemetry.addData("SH_Servo State", angleServo.getState());
        telemetry.addData("SH_Robot Pitch", "%.1f°", robotPitch);
        telemetry.addData("SH_Motor Voltage", "%.1fV", shooter.getVoltage());
        telemetry.addData("SH_Ready to Shoot", isAtTargetSpeed(100) ? "YES" : "NO");
    }

    public void stop() {
        shooter.setPower(0);
        targetRPM = 0;
        integral = 0;
        rpmKalman.reset();
    }

    // Kalman Filter class (unchanged)
    private static class KalmanFilter {
        private double state;
        private double velocity;
        private double uncertainty;
        private double velocityUncertainty;
        private final double processNoise;
        private final double measurementNoise;

        public KalmanFilter(double initialState, double initialUncertainty,
                            double processNoise, double measurementNoise) {
            this.state = initialState;
            this.velocity = 0;
            this.uncertainty = initialUncertainty;
            this.velocityUncertainty = initialUncertainty;
            this.processNoise = processNoise;
            this.measurementNoise = measurementNoise;
        }

        public void predict(double dt, double acceleration) {
            state = state + velocity * dt + 0.5 * acceleration * dt * dt;
            velocity = velocity + acceleration * dt;
            uncertainty += processNoise * dt;
            velocityUncertainty += processNoise * dt;
        }

        public void update(double measurement) {
            double kalmanGain = uncertainty / (uncertainty + measurementNoise);
            double innovation = measurement - state;
            state = state + kalmanGain * innovation;
            velocity = velocity + (kalmanGain * 0.1) * innovation;
            uncertainty = (1 - kalmanGain) * uncertainty;
            if (uncertainty < 1.0) uncertainty = 1.0;
        }

        public double getState() {
            return state;
        }

        public double getUncertainty() {
            return uncertainty;
        }

        public void reset() {
            state = 0;
            velocity = 0;
            uncertainty = 100.0;
        }
    }

    // ✅ CORRECTED GOAL COORDINATES AND HEIGHT
    public static class ProjectileCalculations {
        // Goal coordinates from your specs
        private static final double RED_GOAL_X = 131.0;   // ✅ Corrected
        private static final double RED_GOAL_Y = 137.0;   // ✅ Corrected
        private static final double BLUE_GOAL_X = 13.0;   // ✅ Corrected
        private static final double BLUE_GOAL_Y = 138.0;  // ✅ Corrected

        private static final double GRAVITY = 386.4; // in/s^2
        private static final double WHEEL_RADIUS = 2.0; // inches
        private static final double MAX_MOTOR_RPM = 5000.0;

        // Energy loss compensation - tune this empirically
        private static final double ENERGY_LOSS_MULTIPLIER = 1.30;

        /**
         * Calculates required launch angle and RPM to hit the goal
         * Takes into account robot movement and pitch
         *
         * @param robotX Current X position (inches)
         * @param robotY Current Y position (inches)
         * @param goalX Target goal X position (inches)
         * @param goalY Target goal Y position (inches)
         * @param goalHeight Height of goal basket (inches)
         * @param shooterHeight Height of shooter on robot (inches)
         * @param robotVelTowardGoal Robot velocity toward goal (in/s)
         * @param robotPitch Robot pitch angle from IMU (degrees)
         * @return BallisticsResult with calculated RPM and angle
         */
        public static BallisticsResult calculateBallisticsWithMovement(
                double robotX, double robotY, double goalX, double goalY, double goalHeight,
                double shooterHeight, double robotVelTowardGoal, double robotPitch) {

            // Calculate distances
            double dx = goalX - robotX;
            double dy = goalY - robotY;
            double horizontalDist = Math.sqrt(dx * dx + dy * dy);
            double verticalDist = goalHeight - shooterHeight;

            // Choose launch angle based on distance
            double angle = chooseAngle(horizontalDist, verticalDist);

            // Compensate for robot pitch
            angle -= robotPitch;
            angle = Range.clip(angle, 25, 65);

            double angleRad = Math.toRadians(angle);

            // Calculate initial velocity needed for stationary robot
            double v0_stationary = calculateInitialVelocity(horizontalDist, verticalDist, angleRad);

            // Compensate for robot movement toward/away from goal
            double timeOfFlight = (2.0 * v0_stationary * Math.sin(angleRad)) / GRAVITY;
            double distanceTraveled = robotVelTowardGoal * timeOfFlight;

            // Recalculate for adjusted distance
            double adjustedHorizontalDist = horizontalDist - distanceTraveled;
            adjustedHorizontalDist = Math.max(adjustedHorizontalDist, 12.0);

            double v0_compensated = calculateInitialVelocity(adjustedHorizontalDist, verticalDist, angleRad);
            double v0 = Math.max(v0_compensated, 10.0);

            // Convert velocity to RPM
            double rpm = (v0 * 60.0) / (2.0 * Math.PI * WHEEL_RADIUS);
            rpm *= ENERGY_LOSS_MULTIPLIER;
            rpm = Range.clip(rpm, 800, MAX_MOTOR_RPM);

            return new BallisticsResult(rpm, angle);
        }

        private static double chooseAngle(double horizontalDist, double verticalDist) {
            double angle;
            if (horizontalDist < 60) {
                angle = 50;
            } else if (horizontalDist < 100) {
                angle = 45;
            } else {
                angle = 40;
            }

            if (verticalDist > 12) {
                angle += 5;
            } else if (verticalDist < -12) {
                angle -= 5;
            }

            return Range.clip(angle, 25, 65);
        }

        private static double calculateInitialVelocity(
                double horizontalDist, double verticalDist, double angleRad) {

            double cosTheta = Math.cos(angleRad);
            double tanTheta = Math.tan(angleRad);
            double numerator = GRAVITY * horizontalDist * horizontalDist;
            double denominator = 2.0 * cosTheta * cosTheta *
                    (horizontalDist * tanTheta - verticalDist);

            double velocitySquared;
            if (denominator <= 0) {
                velocitySquared = 500 * 500;
            } else {
                velocitySquared = numerator / denominator;
            }

            if (velocitySquared < 0) {
                velocitySquared = 10000;
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
package org.firstinspires.ftc.teamcode.layered.logical2;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="PID Shooter All Bugs Fixed", group = "default")
public class PIDShooter extends LinearOpMode {
    private DcMotorEx shooter;
    private DcMotorEx lf, lr, rf, rr;
    private Servo angleServo;
    private IMU imu;
    private GoBildaPinpointDriver pinpoint;
    private VoltageSensor voltageSensor;

    // FIXED PID constants - Much smaller feedforward, larger P/I/D
    private final double kP = 0.0012;      // Increased from 0.00047
    private final double kI = 0.000005;    // Increased from 0.000001
    private final double kD = 0.0015;      // Increased from 0.0008
    private final double kF = 0.00012;     // CRITICAL FIX: Was 0.36 (way too high!)

    // State variables
    private double targetRPM = 0;
    private double targetAngle = 0;
    private double integral = 0;
    private double outputPower = 0;
    private double previousRPM = 0;
    private double currentRPM = 0;

    private KalmanFilter rpmKalman;
    private SimplePoseKalman poseKalman;
    private double filteredRPM = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();
    private final ElapsedTime rpmTimer = new ElapsedTime();  // ADDED: Separate timer for RPM
    private boolean isFirstUpdate = true;
    private double lastEncoderPosition = 0;

    // Fallback system
    private boolean pinpointHealthy = true;
    private boolean imuHealthy = true;
    private final ElapsedTime pinpointCheckTimer = new ElapsedTime();
    private double lastPinpointX = 0;
    private double lastPinpointY = 0;
    private double lastPinpointHeading = 0;
    private int pinpointStaleCount = 0;
    private static final int PINPOINT_STALE_THRESHOLD = 5;
    private static final double PINPOINT_MAX_JUMP = 50.0;

    // Auto-aim with PD controller
    private static final double TURN_KP = 0.008;
    private static final double TURN_KD = 0.004;
    private static final double MAX_TURN_POWER = 0.6;
    private static final double HEADING_TOLERANCE = 2.0;
    private double lastHeadingError = 0;
    private final ElapsedTime autoAimTimer = new ElapsedTime();

    // Constants
    private static final double TICKS_PER_REV = 28;
    private static final double MAX_MOTOR_RPM = 6000;
    private static final double INTEGRAL_MAX = 1000;
    private static final double INTEGRAL_MIN = -1000;
    private static final double MIN_DELTA_TIME = 0.005;
    private static final double MAX_ENCODER_JUMP = TICKS_PER_REV * 100;
    private static final double RED_GOAL_HEIGHT = 40;
    private static final double BLUE_GOAL_HEIGHT = 40;
    private static final double SHOOTER_HEIGHT = 8.334;

    private static final double SERVO_MIN_ANGLE = 25;
    private static final double SERVO_MAX_ANGLE = 65;
    private static final double SERVO_MIN_POSITION = 0.17;
    private static final double SERVO_MAX_POSITION = 0.83;
    private static final double SERVO_SMOOTH_FACTOR = 0.3;

    // Shooter control
    private static final double RPM_TOLERANCE = 100;

    // Robot pose tracking
    private double robotX = 72;
    private double robotY = 72;
    private double robotHeading = 0;
    private double robotVelX = 0;
    private double robotVelY = 0;

    private boolean isRedAlliance = false;
    private boolean lastBState = false;

    private boolean shooterEnabled = false;
    private boolean lastAState = false;

    private boolean hasRumbled = false;
    private final ElapsedTime telemetryTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize hardware
        shooter = hardwareMap.get(DcMotorEx.class, "dmot");
        angleServo = hardwareMap.get(Servo.class, "angle_servo");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        lf.setDirection(DcMotorEx.Direction.REVERSE);
        lr.setDirection(DcMotorEx.Direction.REVERSE);
        rf.setDirection(DcMotorEx.Direction.FORWARD);
        rr.setDirection(DcMotorEx.Direction.FORWARD);

        // Initialize Pinpoint with error handling
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            configurePinpoint();
            pinpointHealthy = true;
            telemetry.addData("Pinpoint", "OK");
        } catch (Exception e) {
            pinpoint = null;
            pinpointHealthy = false;
            telemetry.addData("Pinpoint", "FAILED - Using IMU fallback");
        }

        // Initialize IMU with error handling
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            imuHealthy = true;
            telemetry.addData("IMU", "✓ OK");
        } catch (Exception e) {
            imu = null;
            imuHealthy = false;
            telemetry.addData("IMU", "FAILED");
        }

        telemetry.update();

        // Check we have at least one sensor
        if (!pinpointHealthy && !imuHealthy) {
            telemetry.addData("ERROR", "No localization sensors available!");
            telemetry.addData("Action", "Please check sensor connections");
            telemetry.update();
            while (!isStopRequested()) sleep(100);
            return;
        }

        // Initialize robot pose
        if (pinpointHealthy) {
            Pose2D pose = pinpoint.getPosition();
            robotX = pose.getX(DistanceUnit.INCH);
            robotY = pose.getY(DistanceUnit.INCH);
            robotHeading = pose.getHeading(AngleUnit.DEGREES);
            lastPinpointX = robotX;
            lastPinpointY = robotY;
            lastPinpointHeading = robotHeading;
        } else if (imuHealthy) {
            robotX = 72;
            robotY = 72;
            robotHeading = -imu.getRobotOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        }

        // Initialize shooter
        shooter.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rpmKalman = new KalmanFilter(0.0, 100.0, 50.0, 200.0);
        poseKalman = new SimplePoseKalman(robotX, robotY, robotHeading);

        lastEncoderPosition = shooter.getCurrentPosition();
        pidTimer.reset();
        rpmTimer.reset();  // ADDED
        pinpointCheckTimer.reset();
        autoAimTimer.reset();
        angleServo.setPosition(mapAngleToServo(SERVO_MIN_ANGLE));

        telemetry.addData("Status", "Ready - Press START");
        telemetry.addData("Controls", "A=Shooter X=Auto-aim B=Alliance");
        telemetry.update();

        waitForStart();
        autoAimTimer.reset();

        while (opModeIsActive()) {
            double dt = pidTimer.seconds();
            pidTimer.reset();

            // Update localization with fallback
            updateLocalization(dt);

            // Handle gamepad controls
            handleGamepadControls();

            // Update shooter
            if (shooterEnabled) {
                updateShooter(dt);

                // Smooth servo movement
                double targetServoPos = mapAngleToServo(targetAngle);
                double currentServoPos = angleServo.getPosition();
                angleServo.setPosition(smoothServo(currentServoPos, targetServoPos, SERVO_SMOOTH_FACTOR));

                // Rumble when ready to fire
                if (isAtTargetSpeed(RPM_TOLERANCE) && !hasRumbled) {
                    gamepad1.rumble(300);
                    hasRumbled = true;
                } else if (!isAtTargetSpeed(RPM_TOLERANCE)) {
                    hasRumbled = false;
                }
            } else {
                stopShooter();
                hasRumbled = false;
            }

            // Update telemetry
            if (telemetryTimer.seconds() > 0.15) {
                updateTelemetry();
                telemetryTimer.reset();
            }
        }

        stopShooter();
    }

    private void handleGamepadControls() {
        // Auto-aim with PD controller
        if (gamepad1.x) {
            double goalX = isRedAlliance ? ProjectileCalculations.RED_GOAL_X : ProjectileCalculations.BLUE_GOAL_X;
            double goalY = isRedAlliance ? ProjectileCalculations.RED_GOAL_Y : ProjectileCalculations.BLUE_GOAL_Y;

            double desiredHeading = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));
            while (desiredHeading > 180) desiredHeading -= 360;
            while (desiredHeading < -180) desiredHeading += 360;

            double headingError = desiredHeading - robotHeading;
            while (headingError > 180) headingError -= 360;
            while (headingError < -180) headingError += 360;

            double turnPower = 0;
            if (Math.abs(headingError) > HEADING_TOLERANCE) {
                double dt = autoAimTimer.seconds();
                autoAimTimer.reset();

                // PD controller for smooth turning
                double derivative = (headingError - lastHeadingError) / Math.max(dt, 0.001);
                turnPower = TURN_KP * headingError + TURN_KD * derivative;
                turnPower = Range.clip(turnPower, -MAX_TURN_POWER, MAX_TURN_POWER);

                lastHeadingError = headingError;
            } else {
                lastHeadingError = 0;
            }

            applyTurnPower(turnPower);

            // Auto-enable shooter when aiming
            if (!shooterEnabled) shooterEnabled = true;
        } else {
            applyTurnPower(0);
            lastHeadingError = 0;
        }

        // Toggle alliance (B button)
        boolean currentBState = gamepad1.b;
        if (currentBState && !lastBState) {
            isRedAlliance = !isRedAlliance;
            gamepad1.rumble(100);
        }
        lastBState = currentBState;

        // Toggle shooter (A button)
        boolean currentAState = gamepad1.a;
        if (currentAState && !lastAState) {
            shooterEnabled = !shooterEnabled;
            gamepad1.rumble(200);
        }
        lastAState = currentAState;
    }

    private void updateLocalization(double dt) {
        if (dt < 0.001 || dt > 1.0) return;

        double measuredX = robotX;
        double measuredY = robotY;
        double measuredHeading = robotHeading;
        boolean gotValidUpdate = false;

        // Try Pinpoint first
        if (pinpointHealthy && pinpoint != null) {
            try {
                pinpoint.update();
                Pose2D currentPose = pinpoint.getPosition();

                double pinpointX = currentPose.getX(DistanceUnit.INCH);
                double pinpointY = currentPose.getY(DistanceUnit.INCH);
                double pinpointHeading = currentPose.getHeading(AngleUnit.DEGREES);

                // Sanity check for unreasonable jumps
                double dx = pinpointX - lastPinpointX;
                double dy = pinpointY - lastPinpointY;
                double jumpDist = Math.hypot(dx, dy);

                if (jumpDist < PINPOINT_MAX_JUMP || pinpointCheckTimer.seconds() > 1.0) {
                    measuredX = pinpointX;
                    measuredY = pinpointY;
                    measuredHeading = pinpointHeading;

                    lastPinpointX = pinpointX;
                    lastPinpointY = pinpointY;
                    lastPinpointHeading = pinpointHeading;

                    pinpointStaleCount = 0;
                    gotValidUpdate = true;
                    pinpointCheckTimer.reset();
                } else {
                    pinpointStaleCount++;
                    if (pinpointStaleCount > PINPOINT_STALE_THRESHOLD) {
                        pinpointHealthy = false;
                        gamepad1.rumble(500);
                    }
                }
            } catch (Exception e) {
                pinpointHealthy = false;
                gamepad1.rumble(500);
            }
        }

        // Fallback to IMU for heading if Pinpoint fails
        if (!gotValidUpdate && imuHealthy && imu != null) {
            try {
                double imuHeading = -imu.getRobotOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                measuredHeading = imuHeading;
                measuredX = robotX;
                measuredY = robotY;
                gotValidUpdate = true;
            } catch (Exception e) {
                imuHealthy = false;
            }
        }

        // Update Kalman filter
        if (gotValidUpdate) {
            poseKalman.predict(dt);
            poseKalman.update(measuredX, measuredY, measuredHeading);

            robotX = poseKalman.getX();
            robotY = poseKalman.getY();
            robotHeading = poseKalman.getHeading();
            robotVelX = poseKalman.getVelX();
            robotVelY = poseKalman.getVelY();
        }
    }

    private void updateShooter(double deltaTime) {
        if (isFirstUpdate || deltaTime < MIN_DELTA_TIME) {
            isFirstUpdate = false;
            lastEncoderPosition = shooter.getCurrentPosition();
            rpmTimer.reset();  // ADDED
            return;
        }

        double goalX = isRedAlliance ? ProjectileCalculations.RED_GOAL_X : ProjectileCalculations.BLUE_GOAL_X;
        double goalY = isRedAlliance ? ProjectileCalculations.RED_GOAL_Y : ProjectileCalculations.BLUE_GOAL_Y;
        double goalHeight = isRedAlliance ? RED_GOAL_HEIGHT : BLUE_GOAL_HEIGHT;

        double angleToGoal = Math.atan2(goalY - robotY, goalX - robotX);
        double velTowardGoal = robotVelX * Math.cos(angleToGoal) + robotVelY * Math.sin(angleToGoal);

        // Calculate ballistics
        ProjectileCalculations.BallisticsResult ballistics =
                ProjectileCalculations.calculateBallisticsWithMovement(
                        robotX, robotY, goalX, goalY, goalHeight, SHOOTER_HEIGHT, velTowardGoal);

        double rawAngle = ballistics.angle;
        double rawRPM = ballistics.rpm;

        // CRITICAL FIX: Always validate ballistics results before using them
        if (Double.isNaN(rawAngle) || Double.isInfinite(rawAngle)) {
            rawAngle = 45; // Safe default
        }
        rawAngle = Range.clip(rawAngle, 30, 65);

        if (Double.isNaN(rawRPM) || Double.isInfinite(rawRPM) || rawRPM < 1000) {
            rawRPM = 3500; // Safe default
        }
        rawRPM = Range.clip(rawRPM, 1500, MAX_MOTOR_RPM);

        // CRITICAL FIX: Reset integral when target changes significantly
        double rpmChange = Math.abs(rawRPM - targetRPM);
        if (rpmChange > 500) {
            integral = 0;  // Clear accumulated error from previous target
        }

        targetAngle = rawAngle;
        targetRPM = rawRPM;

        // CRITICAL FIX: Separate RPM calculation with its own timer
        double rpmDt = rpmTimer.seconds();
        if (rpmDt > 0.01) {  // Only calculate if enough time passed
            double currentEncoderPosition = shooter.getCurrentPosition();
            double deltaPosition = currentEncoderPosition - lastEncoderPosition;

            if (Math.abs(deltaPosition) < MAX_ENCODER_JUMP) {
                currentRPM = (deltaPosition / TICKS_PER_REV) * 60.0 / rpmDt;
            }
            // Don't reset currentRPM if jump detected - keep last valid value

            lastEncoderPosition = currentEncoderPosition;
            rpmTimer.reset();
        }

        // Filter RPM
        rpmKalman.predict(deltaTime, 0.0);
        rpmKalman.update(currentRPM);
        filteredRPM = rpmKalman.getState();

        // Voltage compensation
        double voltage = getBatteryVoltage();
        double nominalVoltage = 13.0;
        double voltageCompFactor = nominalVoltage / Math.max(voltage, 8.0);

        // CRITICAL FIX: Proper feedforward calculation
        double normalizedRPM = targetRPM / MAX_MOTOR_RPM;
        double compensatedFF = kF * normalizedRPM * voltageCompFactor * targetRPM; // may need to change: compensatedFF = kF * targetRPM * voltageCompFactor

        // PID control with anti-windup
        double error = targetRPM - filteredRPM;

        // CRITICAL FIX: Only accumulate integral when close to target
        if (Math.abs(error) < 1000) {
            integral += error * deltaTime;
        } else {
            integral *= 0.9;  // Decay integral when far from target
        }

        if (targetRPM == 0) integral = 0;
        integral = Range.clip(integral, INTEGRAL_MIN, INTEGRAL_MAX);

        double derivative = -(filteredRPM - previousRPM) / deltaTime;

        double pTerm = kP * error;
        double iTerm = kI * integral;
        double dTerm = kD * derivative;

        outputPower = compensatedFF + pTerm + iTerm + dTerm;
        outputPower = Range.clip(outputPower, 0.0, 1.0);

        if (targetRPM > 0) {
            shooter.setPower(outputPower);
        } else {
            shooter.setPower(0);
            integral = 0;
        }

        previousRPM = filteredRPM;
    }

    private double smoothServo(double currentPos, double targetPos, double smoothingFactor) {
        return currentPos + (targetPos - currentPos) * smoothingFactor;
    }

    private double mapAngleToServo(double angle) {
        angle = Range.clip(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        double normalized = (angle - SERVO_MIN_ANGLE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);
        double servoPos = SERVO_MIN_POSITION + normalized * (SERVO_MAX_POSITION - SERVO_MIN_POSITION);
        return Range.clip(servoPos, SERVO_MIN_POSITION, SERVO_MAX_POSITION);
    }

    private void applyTurnPower(double turnPower) {
        lf.setPower(turnPower);
        lr.setPower(turnPower);
        rf.setPower(-turnPower);
        rr.setPower(-turnPower);
    }

    private double getBatteryVoltage() {
        double minVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 6 && v < minVoltage) minVoltage = v;
        }
        return minVoltage == Double.POSITIVE_INFINITY ? 13.0 : minVoltage;
    }

    private void configurePinpoint() {
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    public boolean isAtTargetSpeed(double tolerance) {
        return targetRPM > 0 && Math.abs(targetRPM - filteredRPM) < tolerance;
    }

    public void stopShooter() {
        shooter.setPower(0);
        targetRPM = 0;
        integral = 0;
    }

    private void updateTelemetry() {
        double voltage = getBatteryVoltage();
        telemetry.clearAll();

        // Sensor status
        String sensorStatus = "";
        if (pinpointHealthy) {
            sensorStatus = "Pinpoint OK";
        } else if (imuHealthy) {
            sensorStatus = "IMU FALLBACK";
        } else {
            sensorStatus = "NO SENSORS";
        }
        telemetry.addData("Localization", sensorStatus);

        double goalX = isRedAlliance ? ProjectileCalculations.RED_GOAL_X : ProjectileCalculations.BLUE_GOAL_X;
        double goalY = isRedAlliance ? ProjectileCalculations.RED_GOAL_Y : ProjectileCalculations.BLUE_GOAL_Y;
        double distToGoal = Math.hypot(goalX - robotX, goalY - robotY);

        // Shooter status
        if (!shooterEnabled) {
            telemetry.addLine("Press 'A' to enable shooter");
        } else {
            String readyStatus = isAtTargetSpeed(RPM_TOLERANCE) ? "READY TO FIRE!" : "Spinning up...";
            telemetry.addData("Status", readyStatus);
        }

        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Position", "X: %.1f  Y: %.1f", robotX, robotY);
        telemetry.addData("Heading", "%.1f°", robotHeading);
        telemetry.addData("Distance to Goal", "%.1f in", distToGoal);
        telemetry.addData("", "");
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Current RPM (raw)", "%.0f", currentRPM);  // Shows raw unfiltered
        telemetry.addData("Filtered RPM", "%.0f", filteredRPM);
        telemetry.addData("RPM Error", "%.0f", targetRPM - filteredRPM);
        telemetry.addData("Motor Power", "%.2f", outputPower);
        telemetry.addData("", "");

        // Debug PID contributions
        double error = targetRPM - filteredRPM;
        double normalizedRPM = targetRPM / MAX_MOTOR_RPM;
        double voltageCompFactor = 13.0 / Math.max(voltage, 8.0);
        telemetry.addData("FF", "%.4f", kF * normalizedRPM * voltageCompFactor * targetRPM);
        telemetry.addData("P term", "%.4f", kP * error);
        telemetry.addData("I term", "%.4f", kI * integral);
        telemetry.addData("D term", "%.4f", kD * -(filteredRPM - previousRPM) / 0.02);
        telemetry.addData("", "");

        telemetry.addData("Target Angle", "%.1f°", targetAngle);
        telemetry.addData("Servo Position", "%.2f", angleServo.getPosition());
        telemetry.addData("Battery", "%.2fV", voltage);
        telemetry.update();
    }

    // Kalman Filter for RPM smoothing
    private static class KalmanFilter {
        private double state;
        private double velocity;
        private double uncertainty;
        private final double processNoise;
        private final double measurementNoise;

        public KalmanFilter(double initialState, double initialUncertainty,
                            double processNoise, double measurementNoise) {
            this.state = initialState;
            this.velocity = 0;
            this.uncertainty = initialUncertainty;
            this.processNoise = processNoise;
            this.measurementNoise = measurementNoise;
        }

        public void predict(double dt, double acceleration) {
            state += velocity * dt + 0.5 * acceleration * dt * dt;
            velocity += acceleration * dt;
            uncertainty += processNoise * dt;
        }

        public void update(double measurement) {
            double kalmanGain = uncertainty / (uncertainty + measurementNoise);
            double innovation = measurement - state;
            state += kalmanGain * innovation;
            velocity += (kalmanGain * 0.1) * innovation;
            uncertainty = (1 - kalmanGain) * uncertainty;
            if (uncertainty < 1.0) uncertainty = 1.0;
        }

        public double getState() { return state; }
        public void reset() { state = 0; velocity = 0; uncertainty = 100.0; }
    }

    // Kalman Filter for pose estimation
    private static class SimplePoseKalman {
        private double x, y, heading;
        private double vx, vy, omega;
        private double xUnc, yUnc, headingUnc;

        private static final double PROCESS_NOISE_POS = 0.5;
        private static final double PROCESS_NOISE_VEL = 2.0;
        private static final double MEAS_NOISE_POS = 1.0;
        private static final double MEAS_NOISE_HEADING = 0.5;

        public SimplePoseKalman(double x0, double y0, double heading0) {
            this.x = x0;
            this.y = y0;
            this.heading = heading0;
            this.vx = 0;
            this.vy = 0;
            this.omega = 0;
            this.xUnc = 10.0;
            this.yUnc = 10.0;
            this.headingUnc = 5.0;
        }

        public void predict(double dt) {
            x += vx * dt;
            y += vy * dt;
            heading += omega * dt;

            while (heading > 180) heading -= 360;
            while (heading < -180) heading += 360;

            xUnc += PROCESS_NOISE_POS * dt;
            yUnc += PROCESS_NOISE_POS * dt;
            headingUnc += PROCESS_NOISE_POS * dt;
        }

        public void update(double measX, double measY, double measHeading) {
            double kx = xUnc / (xUnc + MEAS_NOISE_POS);
            double dx = measX - x;
            x += kx * dx;
            vx += kx * 0.3 * dx;
            xUnc = (1 - kx) * xUnc;
            if (xUnc < 0.1) xUnc = 0.1;

            double ky = yUnc / (yUnc + MEAS_NOISE_POS);
            double dy = measY - y;
            y += ky * dy;
            vy += ky * 0.3 * dy;
            yUnc = (1 - ky) * yUnc;
            if (yUnc < 0.1) yUnc = 0.1;

            double kh = headingUnc / (headingUnc + MEAS_NOISE_HEADING);
            double dh = measHeading - heading;

            while (dh > 180) dh -= 360;
            while (dh < -180) dh += 360;

            heading += kh * dh;
            omega += kh * 0.2 * dh;
            headingUnc = (1 - kh) * headingUnc;
            if (headingUnc < 0.1) headingUnc = 0.1;

            while (heading > 180) heading -= 360;
            while (heading < -180) heading += 360;
        }

        public double getX() { return x; }
        public double getY() { return y; }
        public double getHeading() { return heading; }
        public double getVelX() { return vx; }
        public double getVelY() { return vy; }
        public double getOmega() { return omega; }
    }

    // Projectile calculation utilities
    public static class ProjectileCalculations {
        public static final double RED_GOAL_X = 144;
        public static final double RED_GOAL_Y = 72;
        public static final double BLUE_GOAL_X = 0;
        public static final double BLUE_GOAL_Y = 72;
        private static final double GRAVITY = 386.4;
        private static final double WHEEL_RADIUS = 2.0;
        private static final double MAX_MOTOR_RPM = 6000.0;
        private static final double ENERGY_LOSS_MULTIPLIER = 0.65;  // REDUCED from 0.85

        public static BallisticsResult calculateBallisticsWithMovement(
                double robotX, double robotY, double goalX, double goalY, double goalHeight,
                double shooterHeight, double robotVelTowardGoal) {

            double dx = goalX - robotX;
            double dy = goalY - robotY;
            double horizontalDist = Math.sqrt(dx * dx + dy * dy);
            double verticalDist = goalHeight - shooterHeight;

            // IMPROVED: More granular angle selection
            double angle = chooseAngle(horizontalDist, verticalDist);
            angle = Range.clip(angle, 25, 65);

            double angleRad = Math.toRadians(angle);
            double v0_stationary = calculateInitialVelocity(horizontalDist, verticalDist, angleRad);

            // CRITICAL FIX: Better safety check
            if (Double.isNaN(v0_stationary) || Double.isInfinite(v0_stationary)) {
                v0_stationary = 200; // Safe fallback
            }

            double timeOfFlight = (2.0 * v0_stationary * Math.sin(angleRad)) / GRAVITY;
            double distanceTraveled = robotVelTowardGoal * timeOfFlight;
            double adjustedHorizontalDist = Math.max(horizontalDist - distanceTraveled, 12.0);

            double v0 = calculateInitialVelocity(adjustedHorizontalDist, verticalDist, angleRad);

            // CRITICAL FIX: Better safety check
            if (Double.isNaN(v0) || Double.isInfinite(v0) || v0 < 10) {
                v0 = 200; // Safe fallback
            }
            v0 = Math.max(v0, 10.0);

            double rpm = (v0 * 60.0) / (2.0 * Math.PI * WHEEL_RADIUS);
            rpm *= ENERGY_LOSS_MULTIPLIER;
            rpm = Range.clip(rpm, 800, MAX_MOTOR_RPM);

            return new BallisticsResult(rpm, angle);
        }

        private static double chooseAngle(double horizontalDist, double verticalDist) {
            // IMPROVED: More granular distance brackets with flatter angles
            if (horizontalDist < 25) return Range.clip(50, 25, 65);  // Very close
            if (horizontalDist < 40) return Range.clip(46, 25, 65);  // Close
            if (horizontalDist < 60) return Range.clip(43, 25, 65);  // Medium-close
            if (horizontalDist < 80) return Range.clip(40, 25, 65);  // Medium
            if (horizontalDist < 100) return Range.clip(37, 25, 65); // Medium-far
            if (horizontalDist < 120) return Range.clip(34, 25, 65); // Far
            return Range.clip(32, 25, 65);                            // Very far
        }

        private static double calculateInitialVelocity(double horizontalDist, double verticalDist, double angleRad) {
            double cosTheta = Math.cos(angleRad);
            double tanTheta = Math.tan(angleRad);
            double numerator = GRAVITY * horizontalDist * horizontalDist;
            double denominator = 2.0 * cosTheta * cosTheta * (horizontalDist * tanTheta - verticalDist);

            // CRITICAL FIX: Better safety checks
            if (denominator <= 0 || Double.isNaN(denominator) || Double.isInfinite(denominator)) {
                return 200; // Safe fallback velocity
            }

            double velocitySquared = numerator / denominator;

            if (velocitySquared < 0 || Double.isNaN(velocitySquared) || Double.isInfinite(velocitySquared)) {
                return 200; // Safe fallback velocity
            }

            return Math.sqrt(Math.max(velocitySquared, 10000));
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
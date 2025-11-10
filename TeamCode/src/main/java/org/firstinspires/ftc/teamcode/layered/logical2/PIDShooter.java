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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="PID Shooter Fixed", group = "default")
public class PIDShooter extends LinearOpMode {
    private DcMotorEx shooter;
    private DcMotorEx lf, lr, rf, rr;
    private Servo angleServo;
    private IMU imu;
    private GoBildaPinpointDriver pinpoint;
    private VoltageSensor voltageSensor;

    // PID constants
    private final double kP = 0.00047;
    private final double kI = 0.000001;
    private final double kD = 0.0008;
    private final double kF = 0.45;



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
    private boolean isFirstUpdate = true;
    private double lastEncoderPosition = 0;

    // for auto-aiming
    private static final double TURN_KP = 0.012;   // P-controller for turning
    private static final double MAX_TURN_POWER = 0.6;
    private static final double HEADING_TOLERANCE = 2.0; // degrees

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

    // Shooter control
    private static final double RPM_TOLERANCE = 100;
    private static final double RPM_READY_THRESHOLD = 200; // Don't spin until within this RPM

    // Robot pose tracking (fused)
    private double robotX = 72;
    private double robotY = 72;
    private double robotHeading = 0;
    private double robotVelX = 0;
    private double robotVelY = 0;

    private boolean isRedAlliance = false;
    private boolean lastBState = false;

    private boolean shooterEnabled = false;
    private boolean lastAState = false;

    private boolean hasRumbled = false; // track if we've already rumbled so not uncomfortable for el driver
    private final ElapsedTime telemetryTimer = new ElapsedTime(); // for making sure telemetry isn't constantly updated in loop


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        shooter = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        angleServo = hardwareMap.get(Servo.class, "angle_servo");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        lf.setDirection(DcMotorEx.Direction.REVERSE);
        lr.setDirection(DcMotorEx.Direction.REVERSE);
        rf.setDirection(DcMotorEx.Direction.FORWARD);
        rr.setDirection(DcMotorEx.Direction.FORWARD);


        try {
            imu = hardwareMap.get(IMU.class, "imu");
        } catch (Exception e) {
            imu = null;
            telemetry.addData("Warning", "IMU not found");
        }

        configurePinpoint();

        // Initialize robot pose
        Pose2D pose = pinpoint.getPosition();
        robotX = pose.getX(DistanceUnit.INCH);
        robotY = pose.getY(DistanceUnit.INCH);

        double initialHeading = pose.getHeading(AngleUnit.DEGREES);
        if (imu != null) {
            try {
                initialHeading = -imu.getRobotOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            } catch (Exception ignored) {}
        }
        robotHeading = initialHeading;

        // Initialize shooter
        shooter.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rpmKalman = new KalmanFilter(0.0, 100.0, 50.0, 200.0);
        poseKalman = new SimplePoseKalman(robotX, robotY, robotHeading);

        lastEncoderPosition = shooter.getCurrentPosition();
        pidTimer.reset();
        // Set servo to safe start position
        angleServo.setPosition(mapAngleToServo(SERVO_MIN_ANGLE));

        telemetry.addData("Status", "Ready - Press START to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();
            double dt = pidTimer.seconds();
            pidTimer.reset();

            Pose2D currentPose = pinpoint.getPosition();
            double pinpointX = currentPose.getX(DistanceUnit.INCH);
            double pinpointY = currentPose.getY(DistanceUnit.INCH);
            double pinpointHeading = currentPose.getHeading(AngleUnit.DEGREES);

            double imuHeading = pinpointHeading;
            if (imu != null) {
                try {
                    imuHeading = imu.getRobotOrientation(AxesReference.INTRINSIC,
                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                } catch (Exception ignored) {
                }
            }

            if (dt > 0.001 && dt < 1.0) {
                poseKalman.predict(dt);
                poseKalman.update(pinpointX, pinpointY, imuHeading);

                robotX = poseKalman.getX();
                robotY = poseKalman.getY();
                robotHeading = poseKalman.getHeading();
                robotVelX = poseKalman.getVelX();
                robotVelY = poseKalman.getVelY();
            }
            double HEADING_SMOOTH_FACTOR = 0.15;
            robotHeading = robotHeading * (1 - HEADING_SMOOTH_FACTOR) + poseKalman.getHeading() * HEADING_SMOOTH_FACTOR;
            if (gamepad1.x) { // Hold button to auto-turn
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
                    turnPower = Range.clip(TURN_KP * headingError, -MAX_TURN_POWER, MAX_TURN_POWER);
                }

                if (Math.abs(headingError) < HEADING_TOLERANCE) {
                    applyTurnPower(0);
                }

                applyTurnPower(turnPower);

                // Optionally enable shooter automatically while aiming
                if (!shooterEnabled) {
                    shooterEnabled = true;
                }
            } else {
                // Stop turning if button released
                applyTurnPower(0);
            }


            // Toggle alliance
            boolean currentBState = gamepad1.b;
            if (currentBState && !lastBState) {
                isRedAlliance = !isRedAlliance;
            }
            lastBState = currentBState;

            boolean currentAState = gamepad1.a;
            if (currentAState && !lastAState) {
                shooterEnabled = !shooterEnabled;
                gamepad1.rumble(200);
            }
            lastAState = gamepad1.a;


            if (shooterEnabled) {
                updateShooter(dt);

                double targetServoPos = mapAngleToServo(targetAngle);
                double currentServoPos = angleServo.getPosition();
                angleServo.setPosition(smoothServo(currentServoPos, targetServoPos, 0.2)); // smoothing

                if (isAtTargetSpeed(RPM_TOLERANCE) && !hasRumbled) {
                    gamepad1.rumble(300);
                    hasRumbled = true;
                } else if (!isAtTargetSpeed(RPM_TOLERANCE)) {
                    hasRumbled = false; // reset if RPM drops below threshold
                }
            } else {
                stopShooter();
                hasRumbled = false;
            }

            if (telemetryTimer.seconds() > 0.15) { // update only telemetry if 0.15 seconds patssed
                double voltage = getBatteryVoltage();
                telemetry.clearAll();
                double goalX = isRedAlliance ? ProjectileCalculations.RED_GOAL_X : ProjectileCalculations.BLUE_GOAL_X;
                double goalY = isRedAlliance ? ProjectileCalculations.RED_GOAL_Y : ProjectileCalculations.BLUE_GOAL_Y;
                double distToGoal = Math.hypot(goalX - robotX, goalY - robotY);

                if (!shooterEnabled) {
                    telemetry.addLine("Press 'A' to enable shooter");
                } else {
                    telemetry.addLine("Shooter enabled");
                    telemetry.addLine(isAtTargetSpeed(RPM_TOLERANCE) ? "Target RPM reached, ready to fire" : "Spinning up...");
                }

                telemetry.addData("Shooter", shooterEnabled ? "enabled" : "off");
                telemetry.addData("Ready to Fire", isAtTargetSpeed(RPM_TOLERANCE) ? "Yes" : "Spinning up...");
                telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
                telemetry.addData("X (in)", "%.1f", robotX);
                telemetry.addData("Y (in)", "%.1f", robotY);
                telemetry.addData("Heading (deg)", "%.1f", robotHeading);
                telemetry.addData("Target RPM", "%.0f", targetRPM);
                telemetry.addData("Actual RPM", "%.0f", filteredRPM);
                telemetry.addData("Power", "%.2f", outputPower);
                telemetry.addData("Voltage (V)", "%.2f", voltage);
                telemetry.addData("Target Angle (deg)", "%.1f", targetAngle);
                telemetry.addData("Raw Target Angle", targetAngle);
                telemetry.addData("Mapped Servo Pos", mapAngleToServo(targetAngle));
                telemetry.addData("Servo Position", "%.2f", angleServo.getPosition());
                telemetry.addData("Distance to Goal", "%.1f in", distToGoal);
                telemetry.update();
                telemetryTimer.reset();
            }
        }

        stopShooter();
    }

    private double getBatteryVoltage(){
        double minVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 6 && v < minVoltage) minVoltage = v;
        }
        return minVoltage == Double.POSITIVE_INFINITY ? 13.0 : minVoltage;
    }

    private void applyTurnPower(double turnPower) {
        lf.setPower(turnPower);
        lr.setPower(turnPower);
        rf.setPower(-turnPower);
        rr.setPower(-turnPower);
    }
    private double smoothServo(double currentPos, double targetPos, double smoothingFactor) {
        return currentPos + (targetPos - currentPos) * smoothingFactor;
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

    private void updateShooter(double deltaTime) {
        if (isFirstUpdate || deltaTime < MIN_DELTA_TIME) {
            isFirstUpdate = false;
            lastEncoderPosition = shooter.getCurrentPosition();
            return;
        }

        double goalX = isRedAlliance ? ProjectileCalculations.RED_GOAL_X : ProjectileCalculations.BLUE_GOAL_X;
        double goalY = isRedAlliance ? ProjectileCalculations.RED_GOAL_Y : ProjectileCalculations.BLUE_GOAL_Y;
        double goalHeight = isRedAlliance ? RED_GOAL_HEIGHT : BLUE_GOAL_HEIGHT;

        double angleToGoal = Math.atan2(goalY - robotY, goalX - robotX);
        double velTowardGoal = robotVelX * Math.cos(angleToGoal) + robotVelY * Math.sin(angleToGoal);

        // calculate ballistics
        ProjectileCalculations.BallisticsResult ballistics =
                ProjectileCalculations.calculateBallisticsWithMovement(
                        robotX, robotY, goalX, goalY, goalHeight, SHOOTER_HEIGHT,
                        velTowardGoal);

        double rawAngle = ballistics.angle;
        double rawRPM   = ballistics.rpm;

        if (Double.isNaN(rawRPM) || rawRPM < 2000) rawRPM = 4000;
        if (Double.isNaN(rawAngle) || rawAngle < 30 || rawAngle > 65) rawAngle = 45;

        targetAngle = rawAngle;
        targetRPM   = rawRPM;
        // automatically adjust servo to calculated angle
        double servoPosition = mapAngleToServo(targetAngle);
        angleServo.setPosition(servoPosition);

        // calculate current RPM
        double currentEncoderPosition = shooter.getCurrentPosition();
        double deltaPosition = currentEncoderPosition - lastEncoderPosition;
        if (Math.abs(deltaPosition) > MAX_ENCODER_JUMP) deltaPosition = 0;

        currentRPM = (deltaPosition / TICKS_PER_REV) * 60.0 / deltaTime;
        lastEncoderPosition = currentEncoderPosition;

        // Filter RPM
        rpmKalman.predict(deltaTime, 0.0);
        rpmKalman.update(currentRPM);
        filteredRPM = rpmKalman.getState();

        double voltage = getBatteryVoltage();
        double nominalVoltage = 13.0;
        double voltageCompFacotr = nominalVoltage / Math.max(voltage, 8.0); // limit at 8 V
        double compensatedFF = kF * (targetRPM / MAX_MOTOR_RPM) * voltageCompFacotr; // compensate the feedforward even if battery drops bc we need good rpm and stuff

        // PID control
        double error = targetRPM - filteredRPM;
        integral += error * deltaTime;
        if (targetRPM == 0) integral = 0;
        integral = Range.clip(integral, INTEGRAL_MIN, INTEGRAL_MAX);

        double derivative = -(filteredRPM - previousRPM) / deltaTime;

        outputPower = compensatedFF + (kP * error) + (kI * integral) + (kD * derivative);
        outputPower = Range.clip(outputPower, 0.0, 1.0);

        if (targetRPM > 0) {
            shooter.setPower(outputPower);
        } else {
            shooter.setPower(0);
            integral = 0;
        }

        previousRPM = filteredRPM;
    }

    private double mapAngleToServo(double angle) {
        angle = Range.clip(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

        // Map linearly from angle to servo position
        double normalized = (angle - SERVO_MIN_ANGLE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);
        double servoPos = SERVO_MIN_POSITION + normalized * (SERVO_MAX_POSITION - SERVO_MIN_POSITION);

        return Range.clip(servoPos, SERVO_MIN_POSITION, SERVO_MAX_POSITION);
    }



    public void stopShooter() {
        shooter.setPower(0);
        targetRPM = 0;
        integral = 0;
    }

    // simple 1D Kalman Filter for RPM
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

    // simple Pose Kalman Filter (fuses Pinpoint + IMU)
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

    public static class ProjectileCalculations {
        public static final double RED_GOAL_X = 144;
        public static final double RED_GOAL_Y = 72;
        public static final double BLUE_GOAL_X = 0;
        public static final double BLUE_GOAL_Y = 72;
        private static final double GRAVITY = 386.4;
        private static final double WHEEL_RADIUS = 2.0;
        private static final double MAX_MOTOR_RPM = 6000.0;
        private static final double ENERGY_LOSS_MULTIPLIER = 0.85;

        public static BallisticsResult calculateBallisticsWithMovement(
                double robotX, double robotY, double goalX, double goalY, double goalHeight,
                double shooterHeight, double robotVelTowardGoal) {

            double dx = goalX - robotX;
            double dy = goalY - robotY;
            double horizontalDist = Math.sqrt(dx * dx + dy * dy);
            double verticalDist = goalHeight - shooterHeight;

            // Choose angle based on distance - closer = steeper angle
            double angle = chooseAngle(horizontalDist, verticalDist);
            angle = Range.clip(angle, 25, 65);

            double angleRad = Math.toRadians(angle);
            double v0_stationary = calculateInitialVelocity(horizontalDist, verticalDist, angleRad);
            double timeOfFlight = (2.0 * v0_stationary * Math.sin(angleRad)) / GRAVITY;
            double distanceTraveled = robotVelTowardGoal * timeOfFlight;
            double adjustedHorizontalDist = Math.max(horizontalDist - distanceTraveled, 12.0);
            double v0 = Math.max(calculateInitialVelocity(adjustedHorizontalDist, verticalDist, angleRad), 10.0);

            double rpm = (v0 * 60.0) / (2.0 * Math.PI * WHEEL_RADIUS);
            rpm *= ENERGY_LOSS_MULTIPLIER;
            rpm = Range.clip(rpm, 800, MAX_MOTOR_RPM);
            return new BallisticsResult(rpm, angle);
        }

        private static double chooseAngle(double horizontalDist, double verticalDist) {
            // Automatically choose angle based on distance
            double angle;
            if (horizontalDist < 60) {
                angle = 50;  // Close shots - steep angle
            } else if (horizontalDist < 100) {
                angle = 45;  // Medium shots
            } else {
                angle = 40;  // Far shots - flatter trajectory
            }

            return Range.clip(angle, 25, 65);
        }

        private static double calculateInitialVelocity(double horizontalDist, double verticalDist, double angleRad) {
            double cosTheta = Math.cos(angleRad);
            double tanTheta = Math.tan(angleRad);
            double numerator = GRAVITY * horizontalDist * horizontalDist;
            double denominator = 2.0 * cosTheta * cosTheta * (horizontalDist * tanTheta - verticalDist);
            double velocitySquared = (denominator <= 0) ? 500 * 500 : numerator / denominator;
            return Math.sqrt(Math.max(velocitySquared, 10000));
        }

        public static class BallisticsResult {
            public double rpm, angle;
            public BallisticsResult(double rpm, double angle) {
                this.rpm = rpm;
                this.angle = angle;
            }
        }
    }
}
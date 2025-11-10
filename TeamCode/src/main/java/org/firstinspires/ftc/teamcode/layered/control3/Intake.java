package org.firstinspires.ftc.teamcode.layered.control3;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

public class Intake {
    private DcMotorEx intakeMotor;
    private CRServo sorterServo;
    private DcMotorEx encoder;
    private Rev2mDistanceSensor laserSensor;
    private LynxModule hub;

    public enum INTAKE_STATE {
        IDLE,
        INTAKING,
        REVERSING,
        ROTATING,
        AT_DEPOSIT_POS
    }

    private INTAKE_STATE currentState = INTAKE_STATE.IDLE;

    private int targetPosition = 0;
    private boolean moving = false;

    private ElapsedTime intakeTimer                                          = new ElapsedTime();
    private ElapsedTime rotationTimer                                        = new ElapsedTime();
    private final double INTAKE_TIMEOUT                                      = 15.0;
    private final double INTAKE_POWER                                        = 0.8;
    private final double REVERSE_POWER                                       = -0.5;
    private final double SORTER_POWER                                        = 0.2;
    private final int ROTATION_TICKS                                         = 2400;
    private final double ROTATION_TIMEOUT                                    = 3.0;
    private final double PLATE_EMPTY_THRESHOLD_MM                            = 100.0;
    private static final double NOMINAL_BATTERY_VOLTAGE                      = 12.0;
    private static final double MAX_MOTOR_CURRENT                            = 5.0;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "imot");
        sorterServo = hardwareMap.get(CRServo.class, "transfer1");
        encoder = hardwareMap.get(DcMotorEx.class, "encoder");
        laserSensor = hardwareMap.get(Rev2mDistanceSensor.class, "laser");
        hub = hardwareMap.get(LynxModule.class, "Control Hub");

        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intakeTimer.reset();
        rotationTimer.reset();
    }

    public void startIntaking() {
        if (currentState != INTAKE_STATE.ROTATING) {
            currentState = INTAKE_STATE.INTAKING;
            intakeTimer.reset();
        }
    }

    public void stopIntaking() {
        if (currentState == INTAKE_STATE.INTAKING) {
            currentState = INTAKE_STATE.IDLE;
        }
    }

    public void reverse() {
        currentState = INTAKE_STATE.REVERSING;
    }

    public void startRotation() {
        currentState = INTAKE_STATE.ROTATING;
        rotationTimer.reset();
        moving = false;
    }

    public void returnToIdle() {
        currentState = INTAKE_STATE.IDLE;
    }

    public void reset() {
        currentState = INTAKE_STATE.IDLE;
        moving = false;
        sorterServo.setPower(0);
        intakeMotor.setPower(0);
    }

    public INTAKE_STATE getState() {
        return currentState;
    }

    public boolean isAtDepositPosition() {
        return currentState == INTAKE_STATE.AT_DEPOSIT_POS;
    }

    public boolean isPlateEmpty() {
        return laserSensor.getDistance(DistanceUnit.MM) > PLATE_EMPTY_THRESHOLD_MM;
    }

    public int getEncoderPosition() {
        return encoder.getCurrentPosition();
    }

    private double adjMotorPower(double basePower, double batteryVoltage, double motorCurrent) {
        double voltageCompensation = NOMINAL_BATTERY_VOLTAGE / batteryVoltage;

        double currentCompensation = 1.0;
        if (motorCurrent > MAX_MOTOR_CURRENT) {
            currentCompensation = MAX_MOTOR_CURRENT / motorCurrent;
        }

        double adjustedPower = basePower * voltageCompensation * currentCompensation;
        return Math.max(-1.0, Math.min(1.0, adjustedPower));
    }

    public void update() {
        int currentPos = encoder.getCurrentPosition();
        double plateDistance = laserSensor.getDistance(DistanceUnit.MM);
        boolean plateEmpty = plateDistance > PLATE_EMPTY_THRESHOLD_MM;

        // State machine
        switch (currentState) {
            case IDLE:
                intakeMotor.setPower(0);
                sorterServo.setPower(0);
                moving = false;
                break;

            case INTAKING:
                intakeMotor.setPower(adjMotorPower(INTAKE_POWER,
                        hub.getInputVoltage(VoltageUnit.VOLTS),
                        intakeMotor.getCurrent(CurrentUnit.AMPS)));
                sorterServo.setPower(0);

                if (intakeTimer.seconds() > INTAKE_TIMEOUT) {
                    currentState = INTAKE_STATE.IDLE;
                }
                break;

            case REVERSING:
                intakeMotor.setPower(REVERSE_POWER);
                sorterServo.setPower(-SORTER_POWER);
                break;

            case ROTATING:
                intakeMotor.setPower(0);

                // Start rotation on first entry
                if (!moving) {
                    targetPosition = currentPos + ROTATION_TICKS;
                    sorterServo.setPower(SORTER_POWER);
                    moving = true;
                }

                if (moving && (currentPos >= targetPosition ||
                        plateEmpty ||
                        rotationTimer.seconds() > ROTATION_TIMEOUT)) {
                    sorterServo.setPower(0);
                    moving = false;
                    currentState = INTAKE_STATE.AT_DEPOSIT_POS;
                }
                break;

            case AT_DEPOSIT_POS:
                intakeMotor.setPower(0);
                sorterServo.setPower(0);
                moving = false;
                break;
        }
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("IN_State", currentState);
        telemetry.addData("IN_Motor Power", "%.0f%%", intakeMotor.getPower() * 100);
        telemetry.addData("IN_Encoder Pos", encoder.getCurrentPosition());
        telemetry.addData("IN_Plate Distance", "%.1f mm", laserSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("IN_Plate Empty", isPlateEmpty());
        telemetry.addData("IN_Moving", moving);
        if (moving) {
            telemetry.addData("IN_Target Pos", targetPosition);
        }
    }

    public void stop() {
        intakeMotor.setPower(0);
        sorterServo.setPower(0);
    }
}

//private SRSHub.VL53L5CX tofSensor;
//private SRSHub srsHub;

// srsHub.update();

// double avgDistanceInches = 999;
// boolean currentSensorBlocked = false;
        /*
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
*/

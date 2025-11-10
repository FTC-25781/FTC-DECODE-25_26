package org.firstinspires.ftc.teamcode.layered.control3;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.layered.physical1.CRServoForSorter;
import org.firstinspires.ftc.teamcode.layered.physical1.EncoderForIntake;
import org.firstinspires.ftc.teamcode.layered.physical1.LaserSensorForIntake;


public class SorterServoSubsystem {
    private final LaserSensorForIntake laser;
    private final EncoderForIntake encoder;
    private final CRServoForSorter servo;

    public SorterServoSubsystem(HardwareMap hardwareMap) {
        laser = new LaserSensorForIntake(hardwareMap);
        encoder = new EncoderForIntake(hardwareMap);
        servo = new CRServoForSorter(hardwareMap);
    }

    public boolean isAtPos() {
        if (laser.isAtTraget() || encoder.isAtTarget()) {
            encoder.resetEncoder();
            return true;
        }
        return false;
    }

    public void update() {
        if (isAtPos()) {
            servo.StopServo();
        }
    }

    public void start() {
        servo.StartRotation();
    }
}

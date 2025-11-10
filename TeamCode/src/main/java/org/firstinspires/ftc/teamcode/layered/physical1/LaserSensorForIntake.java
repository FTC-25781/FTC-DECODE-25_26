package org.firstinspires.ftc.teamcode.layered.physical1;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LaserSensorForIntake {
    private Rev2mDistanceSensor laserSensor;
    private final double PLATE_EMPTY_THRESHOLD_MM = 50.0;

    public LaserSensorForIntake(HardwareMap hardwareMap) {
        laserSensor = hardwareMap.get(Rev2mDistanceSensor.class, "laser");
    }

    public boolean isAtTraget() {
        return laserSensor.getDistance(DistanceUnit.MM) > PLATE_EMPTY_THRESHOLD_MM;
    }
}

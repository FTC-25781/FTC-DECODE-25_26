package org.firstinspires.ftc.teamcode.layeredOld.logical2;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


/*
 What is class about?
 */
public class Turret {
    public CRServo servo;
    public TurretTracker turretOrientation;
    public double rotationSpeed = 0.3; // max speed for turret
    public double angleTolerance = Math.toRadians(5); // 5 degrees of angle tolerance
    public Turret(HardwareMap hardwareMap, Follower follower) {
        this.turretOrientation = new TurretTracker(hardwareMap, follower);
        servo = hardwareMap.get(CRServo.class, "turretServo");
    }

    /*
    Calcultes the error of the turret to the goal.
    Sets servo speed originally to 0
    If the error is greater than 5 degrees then
    servo speed is set  + or - 0.3 depending on rotational movement
    if it has to move counter clockwise, -0.3 and vice versa
     */
    public void trackGoal(){
        double error = turretOrientation.calculateError();
        double servoSpeed = 0;
        if(Math.abs(error) > angleTolerance){
            servoSpeed = Math.signum(error) * rotationSpeed;
        }
        else{
            servo.setPower(0);
        }
        servo.setPower(servoSpeed);
    }

    public boolean isOnTarget() {
        double error = turretOrientation.calculateError();
        return Math.abs(error) <= angleTolerance;
    }
}

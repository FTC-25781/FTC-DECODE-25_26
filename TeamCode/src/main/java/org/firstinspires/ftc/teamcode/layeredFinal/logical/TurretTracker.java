package org.firstinspires.ftc.teamcode.layeredFinal.logical;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Get Turret Angle from Encoder and Calculate Error for Turret
public class TurretTracker {

    // Turrent motor encoder
    public DcMotorEx encoder;
    // Using goBilda 6000 RPM motor with 28 Ticks at output shaft.
    // 10 to 130, Tick at turret will 28*13
    public static final double TICKS_PER_180_DEG = 181;
    public static final double DEGREES_PER_TICK = 180.0 / TICKS_PER_180_DEG;

    public Follower follower;

    public final double blueX = 9;
    public final double blueY = 138;

    public final double redX = 137;
    public final double redY = 138;

    public boolean isRed = false;

    public TurretTracker(HardwareMap hardwareMap, Follower follower)
    {
        encoder = hardwareMap.get(DcMotorEx.class, "tmot");
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setDirection(DcMotorEx.Direction.REVERSE);
        this.follower = follower;
    }

    // Angle of Turret with respect to the robot
    public double turretLocalAngle()
    {
        return encoder.getCurrentPosition() * DEGREES_PER_TICK;
    }

    // Angle of Turret with respect to the feild
    public double turretGolbalAngle()
    {
        return Math.toDegrees(follower.getPose().getHeading()) + turretLocalAngle();
    }

    /***
     * Get the angle of the goal with respect to current position of the robot (Not Turret)
      * @return angle in degrees
     */
    public double getAngleToGoal(){
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();

        double targetX = isRed ? redX : blueX;
        double targetY = isRed ? redY : blueY;

        double angleRad = Math.atan2(targetY - robotY, targetX - robotX);
        return Math.toDegrees(angleRad) - Math.toDegrees(follower.getHeading());
    }
    public double calculateDesiredTurretAngle(){
        return (getAngleToGoal() - turretGolbalAngle());
    }
    public void resetEncoder() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}

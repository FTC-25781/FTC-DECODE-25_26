package org.firstinspires.ftc.teamcode.layered.logical2;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.layered.control3.pedroPathing.Constants;

public class Shooter {
    private DcMotorEx shooter_motor;
    public Pose startingPose = new Pose(72, 72, Math.toRadians(45));
//    private Follower follower;

    // Goal positions
    private static final Pose RED_GOAL = new Pose(132, 132, 0);
    private static final Pose BLUE_GOAL = new Pose(12, 132, 0);

    // Auto-align parameters
    private boolean isRedAlliance = true;
    private double alignmentTolerance = Math.toRadians(2);
    private static final double RED_GOAL_OFFSET = Math.toRadians(-1.2);
    private static final double BLUE_GOAL_OFFSET = Math.toRadians(1.2);
    public double NOMINAL_VOLTAGE = 12.66;

    public Shooter(HardwareMap hardwareMap) {
        shooter_motor = hardwareMap.get(DcMotorEx.class, "dmot");
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startingPose);
    }

    public void setAlliance(boolean isRed) {
        this.isRedAlliance = isRed;
    }

    public Pose getTargetGoal() {
        return isRedAlliance ? RED_GOAL : BLUE_GOAL;
    }

    public double getAlignmentOffset() {
        return isRedAlliance ? RED_GOAL_OFFSET : BLUE_GOAL_OFFSET;
    }

    public double calculateAngleToGoal(Follower follower) {
        Pose currentPose = follower.getPose();
        Pose targetGoal = getTargetGoal();

        double deltaX = targetGoal.getX() - currentPose.getX();
        double deltaY = targetGoal.getY() - currentPose.getY();

        double baseAngle = Math.atan2(deltaY, deltaX);

        return baseAngle + getAlignmentOffset();
    }

    public double getAngularError(Follower follower) {
        double targetAngle = calculateAngleToGoal(follower);
        double currentAngle = follower.getPose().getHeading();

        double error = targetAngle - currentAngle;
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;

        return error;
    }

    public boolean isAlignedWithGoal(Follower follower) {
        return Math.abs(getAngularError(follower)) < alignmentTolerance;
    }


    public double calculateTurnPower(Follower follower) {
        double error = getAngularError(follower);

        if (Math.abs(error) < alignmentTolerance) {
            return 0.0;
        }

        double turnSpeed;
        if (Math.abs(error) > Math.toRadians(10)) {
            turnSpeed = 0.4;
        } else {
            turnSpeed = 0.2;
        }

        return Math.copySign(turnSpeed, error);
    }

    public double getDistanceToGoal(Follower follower) {
        Pose currentPose = follower.getPose();
        Pose targetGoal = getTargetGoal();

        double deltaX = targetGoal.getX() - currentPose.getX();
        double deltaY = targetGoal.getY() - currentPose.getY();

        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    public double targetRPM(Follower follower) {
        return getDistanceToGoal(follower);
    }

    public double calculateTargetPower(double targetRPM1, double batteryVoltage) {
        double x = targetRPM1;

        final double C4 = -(3.12278E-10);
        final double C3 = -(1.54591E-8);
        final double C2 = 0.0000301652;
        final double C1 = -0.00158756;
        final double C0 = 0.670979;

        double rawPower =
                (C4 * Math.pow(x, 4)) +
                        (C3 * Math.pow(x, 3)) +
                        (C2 * Math.pow(x, 2)) +
                        (C1 * x) +
                        C0;
            double voltageFactor = NOMINAL_VOLTAGE/ batteryVoltage; // voltage compensation

            double power = rawPower * voltageFactor;

            return Range.clip(power, 0.0, 1.0);
    }

    void setSafePower(DcMotor motor, double targetPower){
        final double SLEW_RATE = 0.2;
        double currentPower = motor.getPower();
        double desiredChange = targetPower - currentPower;
        double limitedChange = Math.max(-SLEW_RATE, Math.min(desiredChange, SLEW_RATE));
        motor.setPower(currentPower + limitedChange);
    }

    public void reverseDepositMotor() {
        setSafePower(shooter_motor, -0.4);
    }

    public void shoot(double pow){
        setSafePower(shooter_motor, pow);
    }

//    public boolean autoAlignAndShoot() {
//        follower.update();
//
//        if (isAlignedWithGoal()) {
//            double rpm = targetRPM();
//            double power = calculateTargetPower(rpm);
//            shoot(power);
//            return true;
//        } else {
//            return false;
//        }
//    }

    public void update(Telemetry telemetry, Follower follower) {
//        follower.update();

        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Target Goal", isRedAlliance ? "(132, 132)" : "(12, 132)");
        telemetry.addData("Alignment Offset", "%.2f deg", Math.toDegrees(getAlignmentOffset()));
        telemetry.addData("Distance to Goal", "%.2f inches", getDistanceToGoal(follower));
        telemetry.addData("Target Angle", "%.2f deg", Math.toDegrees(calculateAngleToGoal(follower)));
//        telemetry.addData("Current Heading", "%.2f deg", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Angular Error", "%.2f deg", Math.toDegrees(getAngularError(follower)));
        telemetry.addData("Aligned", isAlignedWithGoal(follower) ? "YES" : "NO");
        telemetry.addData("Turn Power", "%.3f", calculateTurnPower(follower));
        telemetry.addData("Shooter Power", "%.3f", shooter_motor.getPower());
        telemetry.addData("Target RPM", "%.2f", targetRPM(follower));
//        telemetry.update();
    }
}
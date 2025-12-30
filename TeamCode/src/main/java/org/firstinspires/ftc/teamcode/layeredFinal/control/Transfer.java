package org.firstinspires.ftc.teamcode.layeredFinal.control;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.layeredFinal.logical.Flywheel;
import org.firstinspires.ftc.teamcode.layeredFinal.logical.TransferColorSensor;
import org.firstinspires.ftc.teamcode.layeredFinal.physical.SmartKickServo;

public class Transfer {

    private final SmartKickServo kickers;
    private final Flywheel shooter;
    private final TransferColorSensor colorSensors;

    public final int shootingOrder; // 21 (GPP), 22 (PGP), 23 (PPG)
    private final ShootingStateMachine stateMachine;

    private static final double SHOOTER_MIN_VELOCITY = 1000;

    public Transfer(HardwareMap hardwareMap, int shootingOrder) {
        kickers = new SmartKickServo(hardwareMap);
        shooter = new Flywheel(hardwareMap);
        colorSensors = new TransferColorSensor(hardwareMap);

        this.shootingOrder = shootingOrder;
        this.stateMachine = new ShootingStateMachine(this);
    }

    public boolean isShooterAlive() {
        return shooter.getVelocity() > SHOOTER_MIN_VELOCITY;
    }

    public void kickerUp(int kicker) {
        if (!isShooterAlive()) return;

        switch (kicker) {
            case 1: kickers.setKick1Up(); break;
            case 2: kickers.setKick2Up(); break;
            case 3: kickers.setKick3Up(); break;
        }
    }

    public void lowerAllKickers() {
        kickers.setKick1Down();
        kickers.setKick2Down();
        kickers.setKick3Down();
    }

    public void updateColors() {
        colorSensors.update();
    }

    public int getColor(int sensor) {
        switch (sensor) {
            case 1: return colorSensors.colorOfSensor1();
            case 2: return colorSensors.colorOfSensor2();
            case 3: return colorSensors.colorOfSensor3();
        }
        return 0;
    }

    public void shootInOrder() {
        stateMachine.shootInOrder();
    }

    public void shootSequential() {
        stateMachine.shootSequential();
    }

    public boolean isShootingComplete() {
        return stateMachine.isShootingComplete();
    }
}

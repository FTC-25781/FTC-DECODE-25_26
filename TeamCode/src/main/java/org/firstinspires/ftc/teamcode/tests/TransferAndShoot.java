//package org.firstinspires.ftc.teamcode.tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.teamcode.layeredFinal.control.Intake;
//import org.firstinspires.ftc.teamcode.layeredFinal.control.Transfer;
//import org.firstinspires.ftc.teamcode.layeredFinal.logical.Flywheel;
//
//@TeleOp(name = "Intake + Transfer + Shoot", group = "Tests")
//public class TransferAndShoot extends OpMode {
//    private Transfer t;
//    private Flywheel f;
//    private Intake i;
//
//    @Override
//    public void init() {
//        t = new Transfer(hardwareMap);
//        f = new Flywheel(hardwareMap);
//        i = new Intake(hardwareMap);
//
//        t.update();
//        f.update();
//    }
//
//    @Override
//    public void loop() {
//        if (gamepad1.aWasPressed()) {
//            i.forward();
//        }
//
//        if (gamepad1.bWasPressed()) {
//            t.startKickSequenceRandomly();
//        }
//
//        if (gamepad1.yWasPressed()) {
//            t.startKickSequenceInOrder();
//        }
//
//        if (gamepad1.dpadDownWasPressed()) {
//            f.setVelForFarTip();
//        }
//
//        if (gamepad1.dpadUpWasPressed()) {
//            f.setVelForCloseTip();
//        }
//
//        if (gamepad1.dpadLeftWasPressed()) {
//            t.servo_lower_time -= 0.1;
//        }
//
//        if (gamepad1.dpadRightWasPressed()) {
//            t.servo_lower_time += 0.1;
//        }
//
//        if (gamepad1.leftBumperWasPressed()) {
//            t.time_value -= 0.1;
//        }
//
//        if (gamepad1.rightBumperWasPressed()) {
//            t.time_value += 0.1;
//        }
//
//        t.update();
//        f.update();
//
//        telemetry.addData("t:", t.currentState);
//        telemetry.addData("t id:", t.id);
//        telemetry.addData("f:", f.getVelocity());
//
//        telemetry.addData("time_value", t.time_value);
//        telemetry.addData("servo_lower_value", t.servo_lower_time);
//        telemetry.update();
//    }
//}

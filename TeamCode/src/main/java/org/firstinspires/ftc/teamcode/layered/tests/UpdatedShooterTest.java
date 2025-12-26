package org.firstinspires.ftc.teamcode.layered.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.layered.control3.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.layered.physical1.IntakeMotor;
import org.firstinspires.ftc.teamcode.layered.physical1.ServoForSorter;
import org.firstinspires.ftc.teamcode.layered.physical1.ServoForTransfer;

@TeleOp(name="TestShooter", group = "tests")
public class UpdatedShooterTest extends LinearOpMode {
    private DcMotorEx shooter_motor;
    private GoBildaPinpointDriver pinpoint;
    private ServoForTransfer servo_t;

    Servo angle;

    public double power = 0;
    public boolean lastDPadUp = false;
    public boolean lastDPadDown = false;
    public Pose startingPose = new Pose(72, 72, Math.toRadians(45)); // need to figure this out later because how to transition from auto to teleop
    private Follower follower;
    public double outputPower=0.0;
    public double pos=0.0;
    private ServoForSorter servo;
    private IntakeMotor mot;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;

    private boolean lastX = false;

    private boolean lastB = false;
    @Override
    public void runOpMode() {

        shooter_motor = hardwareMap.get(DcMotorEx.class, "dmot");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        angle = hardwareMap.get(Servo.class, "angle_servo");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        servo_t = new ServoForTransfer(hardwareMap);
        mot = new IntakeMotor(hardwareMap);
        servo = new ServoForSorter(hardwareMap);
//        servo_t = new ServoForTransfer(hardwareMap);
//        encoder = new EncoderForIntake(hardwareMap);
//        shooter = new Shooter(hardwareMap);

        waitForStart();
        follower.startTeleopDrive();

        while (opModeIsActive()) {

            follower.update();

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );
            double targetRPM = Math.sqrt(Math.pow((132 - follower.getPose().getX()), 2) + Math.pow((132 - follower.getPose().getY()), 2));

            if(gamepad1.dpad_up && !lastDPadUp){
                outputPower += 0.01;
            }
            else if (gamepad1.dpad_down && !lastDPadDown){
                outputPower -= 0.01;
            }

            if (gamepad1.right_bumper){
                outputPower = 0.68;
            }else if (gamepad1.right_trigger>0.9){
                outputPower = 0.9;
            }
            else if (gamepad1.left_trigger>0.9){
                outputPower = -0.3;
            }
            else if (gamepad1.left_bumper){
                outputPower = 0;
            }


            if(gamepad1.y) {
                servo_t.moveUp();
            }

            if(gamepad1.a) {
                servo_t.moveDown();
            }

            if (gamepad1.x && !lastX) {
                pos -=0.05;
            }

            if (gamepad1.b && !lastB) {
                pos +=0.05;
            }

            if (gamepad1.dpad_right && !lastDpadRight) {
                servo.GoForwards();
            }

            if (gamepad1.dpad_left && !lastDpadLeft ) {
                servo.GoBackwards();
            }

            shooter_motor.setPower(outputPower);
            angle.setPosition(pos);
            servo_t.update();
            mot.update();
            servo.update(telemetry);
//            shooter.update(telemetry);
            telemetry.addData("Power", outputPower);
            telemetry.addData("Follower Pose X", follower.getPose().getX());
            telemetry.addData("Follower Pose Y", follower.getPose().getY());
            telemetry.addData("Follower Pose Head", follower.getPose().getHeading());
            telemetry.addData("Distance", targetRPM);
            telemetry.update();
            lastDPadDown=gamepad1.dpad_down;
            lastDPadUp=gamepad1.dpad_up;
            lastDpadRight = gamepad1.dpad_right;
            lastDpadLeft = gamepad1.dpad_left;
            lastX = gamepad1.x;
            lastB = gamepad1.b;
        }
    }
    private double calculateTargetPower(double targetRPM1) { // mathematical functions
        double x = targetRPM1;

        final double C4 = 8.89811E-9;
        final double C3 = -0.00000379115;
        final double C2 = 0.000566876;
        final double C1 = -0.0316266;
        final double C0 = 1.24078;

        double power = (C4 * Math.pow(x, 4)) +
                (C3 * Math.pow(x, 3)) +
                (C2 * Math.pow(x, 2)) +
                (C1 * x) +
                C0;

        return Range.clip(power, 0.0, 1.0);
    }
}
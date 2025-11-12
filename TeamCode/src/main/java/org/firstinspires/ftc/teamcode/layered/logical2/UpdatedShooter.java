
package org.firstinspires.ftc.teamcode.layered.logical2;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.layered.control3.pedroPathing.Constants;

@TeleOp(name="shooter")
public class UpdatedShooter extends LinearOpMode {
    private DcMotorEx shooter_motor;
    private GoBildaPinpointDriver pinpoint;
    public double power = 0;
    public boolean lastDPadUp = false;
    public boolean lastDPadDown = false;
    public Pose startingPose = new Pose(72, 72, Math.toRadians(45));
    private Follower follower;
    public double outputPower=0.0;

    @Override
    public void runOpMode() {

        shooter_motor = hardwareMap.get(DcMotorEx.class, "dmot");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

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
            double targetRPM = Math.sqrt(Math.pow(follower.getPose().getX() - 132, 2) + Math.pow(follower.getPose().getY() - 132, 2));

            if(gamepad1.a){
                outputPower = calculateTargetPower(targetRPM);
            }
            else if (gamepad1.b){
                outputPower=gamepad1.right_trigger;
            }

            shooter_motor.setPower(outputPower);

            telemetry.addData("Power", outputPower);
            telemetry.addData("Follower Pose", follower);
            telemetry.addData("Distance", targetRPM);
            telemetry.update();
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



package org.firstinspires.ftc.teamcode.layered.logical2;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.layered.control3.pedroPathing.Constants;

public class GetDistanceToGoal {

    Follower follower;
    boolean isBlue;
    public GetDistanceToGoal(Follower follower, boolean isBlue ) {

        this.follower = follower;
        this.isBlue = isBlue;
    }

    public double getDistanceToGoal(){
        Pose robotPose = follower.getPose();
        double goalX;
        if(isBlue)
            goalX = 12;
        else {
            goalX = 132;
        }
        double goalY = 132;

        double dx = goalX - robotPose.getX();
        double dy = goalY - robotPose.getY();
        double distanceToGoal = Math.sqrt(dx * dx + dy * dy);

        // converting inches to mm
        return distanceToGoal*25.4;
    }
}

package org.firstinspires.ftc.teamcode.layeredFinal.control;

public class ShootingOrderHelper {

    public static int getTargetColorForPosition(int shootingOrder, int position) {
        // 21 = GPP, 22 = PGP, 23 = PPG
        // color: 1 = green, 2 = purple

        if (shootingOrder == 21) {           // GPP
            return (position == 0) ? 1 : 2;
        } else if (shootingOrder == 22) {    // PGP
            return (position == 1) ? 1 : 2;
        } else if (shootingOrder == 23) {    // PPG
            return (position == 2) ? 1 : 2;
        }
        return 0;
    }
}

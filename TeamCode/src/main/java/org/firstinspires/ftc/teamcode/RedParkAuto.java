package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RedParkAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        hwMap hw = new hwMap(this);
        waitForStart();

        int pos = 0;
        //Vision

        hw.turnPID(0.7, 0, 0.7 / 30, 0.00000148148, 0.000038, 2);
        hw.goStraightPID(600, 0.0045, 0.0000023, 0.05, 2000, 0.6);
        if(pos == 2){
            hw.turnPID(0.7, -90, 0.7 / 90, 0.00000148148, 0.000038, 2);
            hw.goStraightPID(600, 0.0045, 0.0000023, 0.05, 2000, 0.6);
        }
        else if(pos == 3){
            hw.turnPID(0.7, -90, 0.7 / 90, 0.00000148148, 0.000038, 2);
            hw.goStraightPID(1200, 0.0045, 0.0000023, 0.05, 2000, 0.6);
        }
    }
}

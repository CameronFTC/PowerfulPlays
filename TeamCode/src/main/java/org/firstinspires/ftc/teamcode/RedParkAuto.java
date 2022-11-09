package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RedParkAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        hwMap hw = new hwMap(this);
        Vision vision = new Vision(this);

        String parkDist = "red";

        while (!(isStarted() || isStopRequested())) {

            //SAMPLE HERE
            parkDist = vision.sample();

            telemetry.addData("Position", parkDist);
            telemetry.addData("Position", "Inside of Left Wheel on right side of tile line");

            telemetry.update();

            idle();
        }

         hw.goStraightPID(600, 0.003, 0.000002, 0.05, 2000, 0.8);
        sleep(100);
         hw.turnPID(-0.8, -90, 0.8/90, 0.022, 0.000038, 2);

        sleep(100);
         hw.goStraightPID(1200, 0.00075, 0.000004, 0.05, 2000, 0.8);

        sleep(100);
         hw.turnPID(0.8, 90, 0.8/90, 0.022, 0.000038, 2);

        sleep(100);

        switch (parkDist) {
            case "red":
                hw.goStraightPID(600, 0.003, 0.000002, 0.05, 2000, 0.8);
                break;
            case "green":
                hw.goStraightPID(1200, 0.00075, 0.000004, 0.05, 2000, 0.8);
                break;
            case "blue":
                hw.goStraightPID(1800, 0.0004, 0.0000003, 0.05, 2000, 0.8);
                break;
        }

//        if(pos == 2){
//            hw.turnPID(0.7, -90, 0.7 / 90, 0.00000148148, 0.000038, 2);
//            hw.goStraightPID(600, 0.0045, 0.0000023, 0.05, 2000, 0.6);
//        }
//        else if(pos == 3){
//            hw.turnPID(0.7, -90, 0.7 / 90, 0.00000148148, 0.000038, 2);
//            hw.goStraightPID(1200, 0.0045, 0.0000023, 0.05, 2000, 0.6);
//        }
    }
}

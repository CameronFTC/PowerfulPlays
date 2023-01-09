package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "Autonomous", name = "LeftSideAuto")
public class LeftSideAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        hwMap hw = new hwMap(this);
        Vision vision = new Vision(this);

        String parkDist = "red";

        while (!(isStarted() || isStopRequested())) {

            //SAMPLE HERE
            parkDist = vision.sampleLeft();

            telemetry.addData("Position", parkDist);
            telemetry.addData("Position", "Inside of Left Wheel on right side of tile line");

            telemetry.update();

            idle();
        }

        while(opModeIsActive()){
            hw.turnPID(-0.675, -90, 0.63/90, 1.1, 0.0006, 4);
            sleep(100);

            hw.goStraightPID2(-170, 0.0037, 0.00000005, 0.05, 4000, -0.7, 1);

            sleep(300);
            // hw.turnPID(-0.7, -45, 0.7/45, 0.03, 0.000038, 2);

            /*hw.turnPID(0.7, 45, 0.012, 0.95, 0.0006, 4);

            hw.outtake(2, 0.7);

            hw.goStraightPID(27, 0.02, 0.0000005, 0.05, 4000, 0.7);

            hw.outtake(0.33, -0.3);

            sleep(100);

            hw.rolll(-1, 2);

            hw.outtake(3, 0.97);

            hw.goStraightPID(-30, 0.013, 0.0000005, .05, 4000, -0.7);*/
            sleep(100);

            //hw.turnPID(-0.6, -45, 0.012, 1.25, 0.0006, 4);

            sleep(100);

            hw.turnPID(0.675, 90, 0.63/90, 1.1, 0.0006, 4);

            hw.goStraightPID(355, 0.0015, 0.0000002, 0.05, 5000, 0.7);

            sleep(1000);

            hw.turnPID(-0.675, -90, 0.65/90, 1.15, 0.0006, 0.8);

            sleep(100);

            //hw.goStraightPID(300, 0.0017, 0.0000002, 0.05, 4000, 0.7);

            //hw.startOuttakeThread(0.7, 3);


            //hw.turnPID(0.675, 90, 0.7/90, 1, 0.0006, 4);
//        sleep(100);
//         hw.turnPID(0.6, 90, 0.6/90, 0.022, 0.000038, 2);
//
//        sleep(100);
//         hw.goStraightPID(-1200, 0.00075, 0.000004, 0.05, 2000, 0.6);
//
//        sleep(100);
//         hw.turnPID(0.6, 90, 0.6/90, 0.022, 0.000038, 2);
//
//        sleep(100);
//
            switch (parkDist) {
                case "blue":
                    break;
                case "green":
                    hw.goStraightPID(180, 0.003, 0.000001, 0.05, 2000, 0.8);
                    break;
                case "red":
                    hw.goStraightPID(375, 0.0004, 0.0000003, 0.05, 2000, 0.8);
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
       // waitForStart();
        //hw.turnPID(0.7, 90, 0.7 / 90, 0.00000148148, 0.000038, 4000);
    }
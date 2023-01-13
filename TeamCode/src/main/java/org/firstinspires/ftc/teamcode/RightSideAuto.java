package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "Autonomous", name = "RightSideAuto")
public class RightSideAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        hwMap hw = new hwMap(this);
        Vision vision = new Vision(this);

        String parkDist = "red";

        while (!(isStarted() || isStopRequested())) {

            //SAMPLE HERE
            parkDist = vision.sample();

            hw.claw.setPosition(0);

            telemetry.addData("Position", parkDist);
            telemetry.addData("Position", "Inside of Left Wheel on right side of tile line");

            telemetry.update();

            idle();
        }

       while(opModeIsActive()){
           hw.goStraightPID2(-1310, 0.0005, 0.0000001, 0.03, 4000, -0.7, -1);

           hw.midGoalThread.start();

           sleep(500);

           hw.turnPID(0.675, -36, 0.62/36, 0.6, 0.0006, 4);

           hw.goStraightPID2(80, 0.008, 0.00005, 0.03, 4000, 0.7, -1);

           sleep(1000);

           hw.claw.setPosition(1);

           sleep(3000);



           hw.turnPID(0.675, 123.5, 0.7/125, 0.75, 0.0006, 4);

           sleep(100);

           hw.stack.start();


           //put arm code

           hw.goStraightPID2(300, 0.0022, 0.00001, 0.001, 4000, 0.7, -1);

           hw.claw.setPosition(0);

           sleep(3000);

           hw.tilt.setPosition(1);
           sleep(100);

           hw.goStraightPID2(-380, 0.0009, 0.0001, 0.001, 4000, -0.7, -1);

           hw.lowGoalThread.start();

           sleep(500);

           hw.turnPID(0.675, -36, 0.62/36, 0.6, 0.0006, 4);

           sleep(200);

           hw.goStraightPID2(80, 0.004, 0.00005, 0.03, 4000, 0.7, -1);

           sleep(100);

           hw.claw.setPosition(1);

           sleep(100);

           hw.goStraightPID2(-80, 0.004, 0.00006, 0.03, 4000, -0.7, -1);

           sleep(100);

           hw.turnPID(0.675, 36, 0.62/36, 0.6, 0.0006, 4);

           sleep(3000);


        telemetry.addData("after this point the robot is making thinsg up", 0);
           /*hw.goStraightPID(-190, 0, 0, 0.0, 4000, -0.7);

           sleep(100)
           // hw.turnPID(-0.7, -45, 0.7/45, 0.03, 0.000038, 2);

           hw.turnPID(-0.7, -45, 0.012, 1.1, 0.0006, 2);

           hw.outtake(2, 0.7);

           hw.goStraightPID(33, 0.02, 0.0000005, 0.05, 4000, 0.7);

           hw.outtake(0.33, -0.3);

           sleep(100);

           //hw.rolll(-1, 2);

           hw.outtake(3, 0.97);

           hw.goStraightPID(-7, 0.035, 0.0000005, .005, 4000, -0.7);
           sleep(100);

           hw.turnPID(0.6, 45, 0.012, 1.25, 0.0006, 4);

           sleep(100);

           hw.turnPID(-0.675, -90, 0.68/90, 1.2, 0.0006, 4);

           hw.goStraightPID(415, 0.0015, 0.0000002, 0.05, 5000, 0.7);

           sleep(100);

           hw.turnPID(0.675, 90, 0.635/90, 1.16, 0.0006, 0.8);

           sleep(100);

           //hw.goStraightPID(300, 0.0017, 0.0000002, 0.05, 4000, 0.7);

           //hw.startOuttakeThread(0.7, 3);*/


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
               case "red":
                   //hw.goStraightPID(165, 0.003, 0.000001, 0.05, 2000, 0.8);
                   //hw.goStrafePID(800, 0.0008, 0.000001, 0.0004, 4000, 0.8);
                   break;

               case "green":
                   //hw.goStrafePID(-800, 0.0008, 0.000001, 0.0004, 4000, -0.8);
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

           break;
       }
    }
}

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
           hw.midGoalThread.start();

           hw.goStraightPID2(630, 0.0007, 0.0000001, 0.028, 4000, 0.55, -1);

           sleep(500);

           hw.turnPID(0.5, 35, 0.5/35, 0.9, 0.0006, 4);

           sleep(1000);

           hw.claw.setPosition(1);

           sleep(1500);

           hw.turnPID(-0.3, -35, 0.3/35, 0.9, 0.0006, 4);

           hw.goStraightPID2(490, 0.0007, 0.0000001, 0.028, 4000, 0.55, -1);

           hw.turnPID(-0.3, -90, 0.3/90, 0.9, 0.0001, 4);

           sleep(100);

           hw.stack.start();


           //put arm code
           hw.goStraightPID2(310, 0.0022, 0.00001, 0.001, 4000, 0.35, -1);

           sleep(100);

           hw.claw.setPosition(0);

           sleep(500);

           //hw.tilt.setPosition(0);

           sleep(500);

           hw.lowGoalThread.start();

           sleep(1000);

           sleep(1000);

           hw.goStraightPID2(-410, 0.002, 0.0001, 0.001, 4000, -0.35, -1);

           sleep(500);

           hw.turnPID(-0.4, -33, 0.4/33, 0.5, 0.0006, 4);

           sleep(700);

           hw.claw.setPosition(1);

           sleep(1000);

           //hw.goStraightPID2(-40, 0.004, 0.00006, 0.03, 4000, -0.7, -1);

           sleep(100);

           hw.turnPID(0.7, 40, 0.65/40, 0.7, 0.0006, 4);

           sleep(3000);


        telemetry.addData("after this point the robot is making thinsg up", 0);


           hw.claw.setPosition(0);
           //hw.tilt.setPosition(1);

           sleep(100);

           hw.arm1.setPosition(0.75);
           hw.arm2.setPosition(0.88);


           switch (parkDist) {
               case "red":
                   hw.goStraightPID2(350, 0.004, 0.0001, 0.001, 4000, -0.7, -1);
                   break;

               case "blue":
                   hw.goStraightPID2(-320, 0.0022, 0.00001, 0.001, 4000, 0.6, -1);
                   break;

               case "green":
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
//211 175
           hw.turnPID(0.5, 90, 0.5/90, 0.6, 0.0006, 1.5);
           hw.goStraightPID2(-220, 0.004, 0.0001, 0.001, 4000, -0.7, -1);
           break;
       }
    }
}

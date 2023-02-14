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
            parkDist = vision.sample();

            hw.claw.setPosition(0);

            telemetry.addData("Position", parkDist);
            telemetry.addData("Position", "Inside of Left Wheel on right side of tile line");

            telemetry.update();

            idle();
        }

        while(opModeIsActive()){
            hw.midGoalThread.start();

            sleep(500);

            hw.goStraightPID2(1160, 0.0007, 0.0000001, 0.028, 4000, 0.55, -1);

            sleep(500);

            hw.autoOuttake(1000, 0.5, 4000);

            hw.turnPID(-0.3, -33.5, 0.3/34, 0.9, 0.0006, 4);

            hw.claw.setPosition(1);

            sleep(1000);

            hw.turnPID(0.6, 122, 0.6/122, 0.9, 0.0006, 4);

            /*
            hw.autoOuttake(700, -0.5, 2000);

            sleep(100);

            hw.stack.start();

            //put arm code
            hw.goStraightPID2(240, 0.0022, 0.00001, 0.001, 4000, 0.35, -1);

            sleep(100);

            hw.claw.setPosition(0);

            sleep(1000);

            hw.tilt.setPosition(0);

            sleep(1000);

            hw.turnPID(0.5, 5, 0.5/5, 0.9, 0.0006, 4);

            hw.goStraightPID2(-720, 0.002, 0.0001, 0.001, 4000, -0.65, -1);

            hw.midGoalThread.start();

            sleep(500);

            hw.autoOuttake(1200, 0.5, 2000);

            hw.turnPID(0.7, 160, 0.7/160, 0.5, 0.0006, 4);

            hw.claw.setPosition(1);

            sleep(1000);

            //hw.goStraightPID2(-40, 0.004, 0.00006, 0.03, 4000, -0.7, -1);

            sleep(100);

            hw.autoOuttake(800, -0.5, 2000);

            hw.turnPID(-0.4, -35, 0.4/35, 0.7, 0.0006, 4);

            sleep(3000);


            telemetry.addData("after this point the robot is making thinsg up", 0);*/


            hw.claw.setPosition(0);
            //hw.tilt.setPosition(1);

            sleep(100);

            hw.arm1.setPosition(0.75);
            hw.arm2.setPosition(0.88);


            switch (parkDist) {
                case "red":
                    hw.goStraightPID2(240, 0.004, 0.0001, 0.001, 4000, -0.7, -1);
                    break;

                case "blue":
                    hw.goStraightPID2(-350, 0.0022, 0.00001, 0.001, 4000, 0.6, -1);
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
            hw.turnPID(-0.5, -90, 0.5/90, 0.6, 0.0006, 1.5);
            hw.goStraightPID2(-220, 0.004, 0.0001, 0.001, 4000, -0.7, -1);
            break;
        }
    }
}
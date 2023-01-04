package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "Autonomous", name = "RightSideAutoStrafe")
public class RightSideAutoWithStrafe extends LinearOpMode {
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

       while(opModeIsActive()){
           hw.goStrafePID(200, 0.004, 0.000001, 0.05, 4000, 0.8);

           sleep(100);

           hw.goStraightPID(415, 0.0015, 0.0000002, 0.05, 5000, 0.7);

           hw.turnPID(-0.7, -45, 0.012, 1.1, 0.0006, 2);

           //outtake
           ////////////hw.arm1.setPosition(0.34); //ARM RAISE WRONG SIDE
           //hw.arm2.setPosition(0.22); //ARM RAISE WRONG SIDE
           //hw.wrist.setPosition(-1); //TILT UP

           //hw.arm1.setPosition(0.53); //.56 //ARM DOWN WRONG SIDE
           //hw.arm2.setPosition(0.53); //0.39 //ARM DOWN WRONG SIDE

           //hw.claw.setPosition(-1); //CLAW OPEN

           //hw.arm1.setPosition(0.34); //ARM RAISE WRONG SIDE
           //hw.arm2.setPosition(0.22); //ARM RAISE WRONG SIDE

           //hw.wrist.setPosition(1); //TILT DOWN

           /*outtake
           hw.claw.setPosition(1); //CLAW CLOSE
           hw.claw.setPosition(-1); //CLAW OPEN

           hw.wrist.setPosition(-1); //TILT UP
           hw.wrist.setPosition(1); //TILT DOWN

           hw.tilt.setPosition(0); //TWIST WRIST
           hw.tilt.setPosition(1); //TWIST WRIST

           hw.arm1.setPosition(eeeee); //ARM RAISE CAMERA SIDE
           hw.arm2.setPosition(eeeee); //ARM RAISE CAMERA SIDE

           hw.arm1.setPosition(0.34); //ARM RAISE WRONG SIDE
           hw.arm2.setPosition(0.22); //ARM RAISE WRONG SIDE

           hw.arm1.setPosition(0.53); //.56 //ARM DOWN WRONG SIDE
           hw.arm2.setPosition(0.53); //0.39 //ARM DOWN WRONG SIDE
           hw.tilt.setPosition(1);

           *///reset

           hw.turnPID(-0.7, -45, 0.012, 1.1, 0.0006, 2);


           /////////hw.arm1.setPosition(0.53); //.56
           //hw.arm2.setPosition(0.53); //0.39
           //hw.tilt.setPosition(1);

           hw.goStraightPID(-300, 0.035, 0.0000005, .005, 4000, -0.7);
           sleep(100);

           //intake

           //////////////////////hw.claw.setPosition(1); //CLAW CLOSE
           //hw.arm1.setPosition(eeeee); //ARM RAISE CAMERA SIDE
           //hw.arm2.setPosition(eeeee); //ARM RAISE CAMERA SIDE







           hw.goStraightPID(200, 0.035, 0.0000005, .005, 4000, -0.7);
           sleep(100);


           hw.turnPID(0.6, -45, 0.012, 1.25, 0.0006, 4);

           //outtake

           //////TO BE CONTINUED

           switch (parkDist) {
               case "red":
                   break;
               case "green":
                   hw.goStraightPID(165, 0.003, 0.000001, 0.05, 2000, 0.8);
                   break;
               case "blue":
                   hw.goStraightPID(360, 0.002, 0.000002, 0.05, 2000, 0.8);
                   break;
           }

           sleep(30000);


       }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class BlueParkAuto extends LinearOpMode {
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

        hw.turnPID(0.8, 90, 0.8 / 90, 0.022, 0.000038, 2);
        sleep(100);

        hw.goStraightPID(600, 0.003, 0.000002, 0.05, 2000, 0.8);
        sleep(100);
        hw.turnPID(-0.8, -90, 0.8 / 90, 0.022, 0.000038, 2);

        sleep(100);
        hw.goStraightPID(1200, 0.00075, 0.000004, 0.05, 2000, 0.8);

        sleep(100);
        hw.turnPID(-0.8, -90, 0.8 / 90, 0.022, 0.000038, 2);

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

    }
       // waitForStart();
        //hw.turnPID(0.7, 90, 0.7 / 90, 0.00000148148, 0.000038, 4000);
    }
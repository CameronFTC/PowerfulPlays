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

            int position = 2400;

            switch (parkDist) {
                case "red":
                    position = 570;
                    hw.turnPID3(0.7, 90, 0.7 / 90, 0.00000148148, 0.000038, 4000);
                    break;

                case "green":
                    position = 1420;
                    break;
                case "blue":
                    position = 2470;
                    break;
            }

        }

    }
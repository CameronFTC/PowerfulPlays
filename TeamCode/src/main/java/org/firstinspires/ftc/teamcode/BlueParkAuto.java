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
          hwMap robot = new hwMap(this);

          String liftPos = "posA";

          while (!(isStarted() || isStopRequested())) {

              //SAMPLE HERE
                telemetry.addData("Position", liftPos);
                telemetry.addData("Position", "Inside of Left Wheel on right side of tile line");

                telemetry.update();

                idle();
            }

            int position = 2400;

            switch (liftPos) {
                case "posA":
                    position = 570;
                    break;
                case "posB":
                    position = 1420;
                    break;
                case "posC":
                    position = 2470;
                    break;



            }

        }

    }
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Detector Test", group="test")
public class DetectorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        System.out.println("hello wrld");
        Detector detector = new Detector(hardwareMap);
        telemetry.addData("initializing", "done");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                detector.loadImage();
            }
            // Default position is NONE so make sure you click 'A' to get the actual position.
            Detector.ElementPosition pos = detector.getElementPosition();
            telemetry.addData("seen objects", pos == Detector.ElementPosition.LEFT ? "left" :
                    (pos == Detector.ElementPosition.RIGHT ? "right" : "none"));
            telemetry.addData("H: ", Detector.hVal);
            telemetry.addData("S: ", Detector.sVal);
            telemetry.addData("V: ", Detector.vVal);
            telemetry.addData("Pixels Detected: ", Detector.numPixels);
            telemetry.addData("Average X: ", Detector.avg);
            //telemetry.addData("Angle: ", )
            telemetry.update();
        }
    }
}

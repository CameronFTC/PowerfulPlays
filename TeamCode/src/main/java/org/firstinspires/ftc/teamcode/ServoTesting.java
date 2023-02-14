package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hwMap;

import java.util.List;

@TeleOp(name = "Servo Tests", group = "Concept")
public class ServoTesting extends LinearOpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    hwMap hw;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */


    private Runnable liftUp = new Runnable() {
        @Override
        public void run() {
            //lift();
        }
    };

    private double currGyro;


    @Override
    public void runOpMode() {
        hw = new hwMap(this);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        hw.liftEncoderGlobal = hw.lift.getCurrentPosition();
        currGyro =  hw.getAngle();


        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        double error = 0;

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                        trigMecanum(error);
                        liftNow();
                        //rollers();
                        claw();
                        //wrist();
                        arms();
                        macros();
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */

    private void claw(){
        if(gamepad2.right_bumper){
            hw.claw.setPosition(-1);
            telemetry.addLine("claw trying now open");
            telemetry.update();
        }
        else if (gamepad2.left_bumper){
            hw.claw.setPosition(1);
            telemetry.addLine("claw not trying");
            telemetry.update();
        }
    }


    /*private void wrist(){
        if(gamepad2.dpad_up)
            hw.tilt.setPosition(0.36);

        //0.4 = mid goal writs position


        if(gamepad2.dpad_down)
            hw.tilt.setPosition(0.8);
    }*/

    private void macros(){

        if(gamepad1.a){
            hw.midGoalThread.interrupt();
            hw.lowGoalThread.interrupt();
            hw.pickUpThread.start();

        } else if(gamepad1.b){
            hw.pickUpThread.interrupt();
            hw.midGoalThread.interrupt();
            hw.lowGoalThread.start();

        } else if(gamepad1.y){
            hw.pickUpThread.interrupt();
            hw.lowGoalThread.interrupt();
            hw.midGoalThread.start();
        }
    }

    private void arms() {
        if (gamepad2.b) {
            hw.arm1.setPosition(1); // ground pick up
            hw.arm2.setPosition(1);

        }
        else if (gamepad2.a) {
            hw.arm1.setPosition(0); //past high score
            hw.arm2.setPosition(0);

        }

        else if (gamepad2.x) {
            hw.arm1.setPosition(0.5); //low pick up
            hw.arm2.setPosition(0.65); //0.39

        }

        else if (gamepad2.y) {
            hw.arm1.setPosition(0.35); //mid pick up
            hw.arm2.setPosition(0.5); //0.39
            //hw.tilt.setPosition(1);

            //down picking up wrong side

            //hw.arm1.setPosition(0.505)


            //hw.arm2.setPosition(0.5);
            telemetry.addLine("arm at mid");
            telemetry.update();
        }

        else {
            telemetry.addData("arm1 position", hw.arm1.getPosition());
            telemetry.addData("arm2 position", hw.arm2.getPosition());

        }

    }

    private void liftNow(){
        //telemetry.addLine("lift is working!!!!!! woo hoo");
        if(gamepad2.left_stick_y > 0.1){
            hw.lift.setPower(-gamepad2.left_stick_y);
            hw.lift2.setPower(-gamepad2.left_stick_y);
        }
        else{
            hw.lift.setPower(-gamepad2.left_stick_y * 0.6);
            hw.lift2.setPower(-gamepad2.left_stick_y * 0.6);
        }


        //hw.lift.setPower(0.5);
        double target;
        /*if(gamepad2.right_stick_y > 0){
            target = gamepad2.right_stick_y * 2940;
            double speed = 0.0;
            double start = hw.lift.getCurrentPosition();

            while((hw.lift.getCurrentPosition() < (0.5 * target)) && speed < 0.9){
                speed = (hw.lift.getCurrentPosition() - start) * 0.0005;
                hw.lift.setPower(speed);
                hw.lift2.setPower(speed);
            }

            double encoderLeft = hw.lift.getCurrentPosition() - hw.liftEncoderGlobal;

            while((target - hw.lift.getCurrentPosition()) > encoderLeft){
                speed = 0.8;
                hw.lift.setPower(speed);
                hw.lift2.setPower(speed);
            }

            while((hw.lift.getCurrentPosition() < (0.5 * target)) && speed > 0){
                speed = (target - hw.lift.getCurrentPosition()) * 0.0005;
                hw.lift.setPower(speed);
                hw.lift2.setPower(speed);
            }

            hw.lift.setPower(0);
            hw.lift2.setPower(0);

        } else {
            hw.lift.setPower(0);
            hw.lift2.setPower(0);
        }

        double speed = 0;
*/



    }

    /*private void rollers(){
            hw.roller1.setPower(-(gamepad2.right_trigger - gamepad2.left_trigger));
            hw.roller2.setPower(-(gamepad2.right_trigger - gamepad2.left_trigger));
    }*/

    public double getHeading(double width, double height, double col, double row){
        //FIND CONSTANT
        double heading = Math.toDegrees(Math.atan(((Math.abs(270 - col)/96.0)/ (-5.904 * Math.pow(10, -5) * Math.pow(width, 3) + 0.019 * Math.pow(width, 2) - 2.069 * width + 89.97))));

        if (col < 270)
            heading *= -1;

        return -heading;
    }

    boolean turn = false;

    public void trigMecanum(double error) {

        double gyroCorrect = (hw.getTrueDiff(currGyro)) * 0.01;

        if(Math.abs(gamepad1.right_stick_x) > 0 || turn){

            currGyro = hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;

            turn = true;
        }

        if(Math.abs(gamepad1.left_stick_y) > 0){
            turn = false;
        }

        telemetry.addData("gyro correct: " , gyroCorrect);
        telemetry.addData("currGyro " , currGyro);
        telemetry.addData("current angle " , hw.getAngle());

        telemetry.update();

        hw.fL.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.right_stick_x *  0.75)) * 0.65);
        hw.fR.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.right_stick_x * 0.75)) * 0.65);
        hw.bL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.right_stick_x * 0.75)) * 0.65);
        hw.bR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x * 0.75)) * 0.65);

        /*double rightstickx = Math.abs(gamepad1.right_stick_x) * -gamepad1.right_stick_x ;
        double leftstickx = -gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);

        double leftstickyfront = Math.abs(gamepad1.left_stick_y) * gamepad1.left_stick_y ;
        double leftstickyback = Math.abs(gamepad1.left_stick_y) * -gamepad1.left_stick_y ;

        double rFront = Math.hypot(rightstickx, leftstickyfront);
        double rBack = Math.hypot(rightstickx, leftstickyback);

        double robotAngleFront = Math.atan2(leftstickyfront, rightstickx) - Math.PI / 4;
        double robotAngleBack = Math.atan2(leftstickyback, rightstickx) - Math.PI / 4;

        double rightX = leftstickx;

        final double v1 = rFront * Math.cos(robotAngleFront) + rightX;
        final double v2 = rFront * Math.sin(robotAngleFront) - rightX;
        final double v3 = rBack * Math.sin(robotAngleBack) + rightX;
        final double v4 = rBack * Math.cos(robotAngleBack) - rightX; */

        /*
        telemetry.addData("fl", v1);
        telemetry.addData ("fR", v2);
        telemetry.addData ("bL", v3);
        telemetry.addData ("bR", v4);
        telemetry.addData("leftX", gamepad1.left_stick_x);
        telemetry.addData("leftY", gamepad1.left_stick_y);
        telemetry.addData ("Right X", rightX);
        telemetry.update();
        */

        //hw.fL.setPower(v1 + error);
        //hw.fR.setPower(-v2 + error + currGyro);
        //hw.bL.setPower(-v3 + error);// * .79);
        //hw.bR.setPower(v4+ error + currGyro);// * .79);
    }
}

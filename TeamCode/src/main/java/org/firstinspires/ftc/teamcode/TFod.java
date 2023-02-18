package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp(name = "TFod", group = "Concept")
public class TFod extends LinearOpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    hwMap hw;
    private static final String TFOD_MODEL_ASSET = "BlueConeModel.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "Blue_Cone"
    };

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

    private static final String VUFORIA_KEY =
            "AYJgkfL/////AAABmcsn/oH17E7rnb8XOyK+cQ9M7dpMRcwsAJNBt7Alqtqd7YIuijBbIlVZFA2yOOS95e7ZssiuuHUvDuw4W/7YfOPV6YW4gsBeuzq1/lKxn8i1PmomRFm9QrnCIoUWmvkgavhv+HGcO1om007ZIm+S62AwAmmqRrbJ8De2eOoOa1n5i2EZg2belVSaD158qE8RkJkaV9Nv38c/DyVDRsXiV6mlGU79TOAYasMix/dTA9VZuOzsaB+kUgM2bchMY8q5pEOehASGo3+P5TiYhAOsJ4SDLGnOt8JyUL1JbFyNkFjvJkna4rtCa6HkJB0DV9Nh84dZmQ8gkLAtfSqaLyY6KD32TytXC5XfFzdRhLsrVnRt";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */

    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    private Runnable liftUp = new Runnable() {
        @Override
        public void run() {
            //lift();
        }
    };

    private double currGyro;

    public enum LiftHeight{
        TEST,
        GROUND,
        LOW,
        MEDIUM,
        HIGH,
        STACK
    };

    final int lowHeight = 600;
    final int mediumHeight = 900;
    final int highHeight = 2000;
    final int stackHeight = 500;


    @Override
    public void runOpMode() {
        hw = new hwMap(this);
        LiftHeight lift = LiftHeight.TEST;
        ElapsedTime liftTimer = new ElapsedTime();
        hw.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        hw.liftEncoderGlobal = hw.lift.getCurrentPosition();
        currGyro =  hw.getAngle();


        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        liftTimer.reset();
        waitForStart();

        double col = 0;
        double row = 0;
        double width = 1;
        double height = 0;
        double error = 0;

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        if (updatedRecognitions.size() > 0) {

                            Recognition recognition = updatedRecognitions.get(0);

                            col = (recognition.getLeft() + recognition.getRight()) / 2;
                            row = (recognition.getTop() + recognition.getBottom()) / 2;
                            width = Math.abs(recognition.getRight() - recognition.getLeft());
                            height = Math.abs(recognition.getTop() - recognition.getBottom());

                            telemetry.addData("", " ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                            telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);

                            telemetry.addData("error from object:", getHeading(width, height, col, row));
                            telemetry.addData("side 1: ", Math.abs(270 - col) / 96.0);
                            telemetry.addData("side 2: ", (-6.3448878 + (4208.05466 / width)));

                            if (gamepad1.a) {
                                error = getHeading(width, height, col, row) * -0.026;
                            }


                        } else {
                            error = 0;

                        }
                        //-0.00005904x^5 + 0.19x^2 -2.069X + 89.97
                    } else {
                        error = 0;
                    }
                }

                        trigMecanum(error);
                        liftNow();
                        //rollers();
                        claw();
                        //hw.pickUpThread.start();
                        //hw.pickUpThread.start();

                        macros();

                        switch(lift){
                            case TEST:
                                break;
                            case GROUND:
                                if(gamepad2.y) {
                                    lift = LiftHeight.HIGH;
                                }
                                else if(gamepad2.b) {
                                    lift = LiftHeight.MEDIUM;
                                }
                                else if(gamepad2.x) {
                                    lift = LiftHeight.STACK;
                                }
                                else if(gamepad2.a) {
                                    lift = LiftHeight.LOW;
                                }
                                break;
                            case LOW:
                                if(Math.abs(lowHeight - hw.lift.getCurrentPosition()) > 30){
                                    hw.lift.setPower(1 * Math.signum(lowHeight - hw.lift.getCurrentPosition()));
                                    hw.lift2.setPower(1 * Math.signum(lowHeight - hw.lift.getCurrentPosition()));
                                }
                                else{
                                    hw.lift.setPower(0);
                                    hw.lift2.setPower(0);
                                    lift = LiftHeight.GROUND;
                                }
                                break;
                            case MEDIUM:
                                if(Math.abs(mediumHeight - hw.lift.getCurrentPosition()) > 30){
                                    hw.lift.setPower(1 * Math.signum(mediumHeight - hw.lift.getCurrentPosition()));
                                    hw.lift2.setPower(1 * Math.signum(mediumHeight - hw.lift.getCurrentPosition()));
                                }
                                else{
                                    hw.lift.setPower(0);
                                    hw.lift2.setPower(0);
                                    lift = LiftHeight.GROUND;
                                }
                                break;
                            case HIGH:
                                if(Math.abs(highHeight - hw.lift.getCurrentPosition()) > 30){
                                    hw.lift.setPower(1 * Math.signum(highHeight - hw.lift.getCurrentPosition()));
                                    hw.lift2.setPower(1 * Math.signum(highHeight - hw.lift.getCurrentPosition()));
                                }
                                else{
                                    if(!(Math.abs(gamepad2.left_stick_y) > 0.1 || Math.abs(gamepad2.right_stick_y) > 0.1)){
                                        hw.lift.setPower(0.1);
                                        hw.lift2.setPower(0.1);
                                    }
                                    else{
                                        hw.lift.setPower(0);
                                        hw.lift2.setPower(0);
                                        lift = LiftHeight.GROUND;
                                    }
                                }
                                break;
                            case STACK:
                                if(Math.abs(stackHeight - hw.lift.getCurrentPosition()) > 30){
                                    hw.lift.setPower(1 * Math.signum(stackHeight - hw.lift.getCurrentPosition()));
                                    hw.lift2.setPower(1 * Math.signum(stackHeight - hw.lift.getCurrentPosition()));
                                }
                                else{
                                    hw.lift.setPower(0);
                                    hw.lift2.setPower(0);
                                    lift = LiftHeight.GROUND;
                                }
                                break;
                        }

                        telemetry.addData("Bl", hw.bL.getCurrentPosition());
                        telemetry.addData("Br", hw.bR.getCurrentPosition());
                        telemetry.addData("Fl", hw.fL.getCurrentPosition());
                        telemetry.addData("FR", hw.fR.getCurrentPosition());
                        telemetry.addData("lift: ", hw.lift.getCurrentPosition());

                }
            }
        }

    /**
     * Initialize the Vuforia localization engine.
     */

    private void claw(){
        if(gamepad2.right_bumper){
            hw.claw.setPosition(0.32);
            telemetry.addLine("claw trying now open");
            telemetry.update();
        }
        else if (gamepad2.left_bumper){
            hw.claw.setPosition(1);
            telemetry.addLine("claw not trying");
            telemetry.update();
        }
    }

   /* private void wrist(){
        if(gamepad2.x)
            hw.tilt.setPosition(0);

        if(gamepad2.y)
            hw.tilt.setPosition(1);
    }*/

    private void arms() {
        if (gamepad2.b) {
            hw.arm1.setPosition(0.1);
            hw.arm2.setPosition(0.1);
            //hw.tilt.setPosition(0);
            telemetry.addLine("arm1 at 0.5");
            telemetry.update();

            //up scoring on wrong side
        }
        else if (gamepad2.a) {
            hw.arm1.setPosition(1); //.56
            hw.arm2.setPosition(1); //0.39
            //hw.tilt.setPosition(1);

            //down picking up wrong side

            //hw.arm1.setPosition(0.505)


            //hw.arm2.setPosition(0.5);
            telemetry.addLine("arm2 at 0.5");
            telemetry.update();
        }

        else {
            telemetry.addData("arm1 position", hw.arm1.getPosition());
            telemetry.addData("arm2 position", hw.arm2.getPosition());

        }

    }

    private void liftNow(){
        //telemetry.addLine("lift is working!!!!!! woo hoo");

        if(gamepad2.left_stick_y < 0.1){
            hw.lift.setPower(gamepad2.left_stick_y);
            hw.lift2.setPower(gamepad2.left_stick_y);
        }
        else if(gamepad2.left_stick_y > -0.1){
            hw.lift.setPower(gamepad2.left_stick_y * 0.6);
            hw.lift2.setPower(gamepad2.left_stick_y * 0.6);
        }
        else if(gamepad2.right_stick_y > 0.1){
            hw.lift.setPower(gamepad2.right_stick_y * 0.3);
            hw.lift2.setPower(gamepad2.right_stick_y * 0.3);
        }
        else{
            hw.lift.setPower(gamepad2.right_stick_y * 0.15);
            hw.lift2.setPower(gamepad2.right_stick_y * 0.15);
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

    private void macros(){

        if(gamepad2.a){
            hw.midGoalThread.interrupt();
            hw.lowGoalThread.interrupt();
            hw.resetThread.interrupt();
            hw.pickUpThread.start();

        } else if(gamepad2.b){
            hw.pickUpThread.interrupt();
            hw.midGoalThread.interrupt();
            hw.resetThread.interrupt();
            hw.lowGoalThread.start();

        } else if(gamepad2.y){
            hw.pickUpThread.interrupt();
            hw.lowGoalThread.interrupt();
            hw.resetThread.interrupt();
            hw.midGoalThread.start();

        }  else if(gamepad2.x){
            hw.pickUpThread.interrupt();
            hw.lowGoalThread.interrupt();
            hw.midGoalThread.interrupt();
            hw.resetThread.start();
        }

    }

    /*private void rollers(){
            hw.roller1.setPower(-(gamepad2.right_trigger - gamepad2.left_trigger));
            hw.roller2.setPower(-(gamepad2.right_trigger - gamepad2.left_trigger));
    }*/

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    public double getHeading(double width, double height, double col, double row){
        //FIND CONSTANT
        double heading = Math.toDegrees(Math.atan(((Math.abs(270 - col)/96.0)/ (-6.3448878 + (4208.05466/width)))));
        if (col < 270)
            heading *= -1;

        return -heading;
    }

    //(-5.904 * Math.pow(10, -5) * Math.pow(width, 3) + 0.019 * Math.pow(width, 2) - 2.069 * width + 89.97))));


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

        hw.fL.setPower(-(-gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.right_stick_x) * 0.65) * 0.75 + error + gamepad2.right_stick_x * 0.3);
        hw.fR.setPower(-(-gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.right_stick_x) * 0.65) * 0.75 - error - gamepad2.right_stick_x * 0.3);
        hw.bL.setPower(-(-gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x) * 0.65) * 0.8 + error + gamepad2.right_stick_x * 0.3);
        hw.bR.setPower(-(-gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.right_stick_x) * 0.65) * 0.75 - error - gamepad2.right_stick_x * 0.3);

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

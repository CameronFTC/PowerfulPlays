package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@TeleOp(group = "TeleOp", name = "PPTeleOp")
public class PPTeleOp extends LinearOpMode {
    public DcMotor fL;
    public DcMotor fR;
    public DcMotor bR;
    public DcMotor bL;

    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    public Orientation startPos = null;
    public double lastDegrees;
    public double globalAngle;
    public double referenceAngle;

    private double currGyro;


    @Override
    public void runOpMode() {
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = this.hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        waitForStart();


        while(opModeIsActive()){
            trigMecanum();

            /*if(Math.abs(gamepad1.right_stick_y) > 0.1){
                fL.setPower(gamepad1.right_stick_y);
            }
            if(Math.abs(gamepad1.left_stick_y) > 0.1){
                fR.setPower(gamepad1.right_stick_y);
            }
            if(Math.abs(gamepad1.right_stick_x) > 0.1){
                bL.setPower(gamepad1.right_stick_y);
            }
            if(Math.abs(gamepad1.left_stick_x) > 0.1){
                bR.setPower(gamepad1.right_stick_y);
            }*/

            /*if(gamepad1.right_trigger > 0.1){
                hw.roller1.setPower(gamepad1.right_trigger);
                hw.roller2.setPower(-gamepad1.right_trigger);
            }
            else if(gamepad1.left_trigger > 0.1){
                hw.roller1.setPower(-gamepad1.left_trigger);
                hw.roller2.setPower(gamepad1.left_trigger);
            }
            else{
                hw.roller1.setPower(0);
                hw.roller2.setPower(0);
            }*/

        }
    }



    public double getHeading(double width, double height, double col, double row){
        //FIND CONSTANT
        double heading = Math.toDegrees(Math.atan(((Math.abs(270 - col)/96.0)/ (-5.904 * Math.pow(10, -5) * Math.pow(width, 3) + 0.019 * Math.pow(width, 2) - 2.069 * width + 89.97))));

        if (col < 270)
            heading *= -1;

        return -heading;
    }

    public void trigMecanum() {

        double gyroCorrect = (getAngle() - currGyro) * 0.01;

        if(Math.abs(gamepad1.right_stick_x) > 0){
            currGyro = getAngle();
        }

        double rightstickx = Math.abs(gamepad1.right_stick_x) * -gamepad1.right_stick_x ;
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
        final double v4 = rBack * Math.cos(robotAngleBack) - rightX;

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

        fL.setPower(v1);
        fR.setPower(-v2 + currGyro);
        bL.setPower(-v3);// * .79);
        bR.setPower(v4 + currGyro);// * .79);

    }

    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return -globalAngle;
    }


}

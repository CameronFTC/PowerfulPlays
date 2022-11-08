package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class hwMap {

    /*
        silly little to do list
        -roadrunner


     */
    public DcMotor bR;
    public DcMotor fR;
    public DcMotor fL;
    public DcMotor bL;
    public DcMotor turret;
    public DcMotor lift;
    public DcMotor lift2;

    public CRServo arm1;
    public CRServo arm2;
    public CRServo roller1;
    public CRServo roller2;

    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    public Orientation startPos = null;
    public double lastDegrees;
    public double globalAngle;
    public double referenceAngle;

    public double liftEncoderGlobal;

    double pwr;
    double time;

    private Runnable autoOuttake = new Runnable()
    {
        @Override
        public void run() {
            outtake(pwr, time);
        }
    };



    LinearOpMode opmode;
    ElapsedTime runtime = new ElapsedTime();

    private Thread turretTurn;
    private Thread outtakeThread;
    private Thread liftThread;

    public hwMap(LinearOpMode opmode) {
        this.opmode = opmode;
        fL = opmode.hardwareMap.get(DcMotor.class, "fL");
        fR = opmode.hardwareMap.get(DcMotor.class, "fR");
        bL = opmode.hardwareMap.get(DcMotor.class, "bL");
        bR = opmode.hardwareMap.get(DcMotor.class, "bR");
        arm1 = opmode.hardwareMap.get(CRServo.class, "arm1");
        arm2 = opmode.hardwareMap.get(CRServo.class, "arm2");

        turret = opmode.hardwareMap.get(DcMotor.class, "turret");
        lift = opmode.hardwareMap.get(DcMotor.class, "lift");
        lift2 = opmode.hardwareMap.get(DcMotor.class, "lift2");

        //lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftEncoderGlobal = 0;

        roller1 = opmode.hardwareMap.get(CRServo.class, "roller1");
        roller2 = opmode.hardwareMap.get(CRServo.class, "roller2");

        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        fL.setDirection(DcMotorSimple.Direction.REVERSE);


        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = opmode.hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        //turretTurn = new Thread(turningTurret);
        outtakeThread = new Thread(autoOuttake);


        //driveTrain.srvMarker.setPosition(1);

        opmode.telemetry.addData("Mode", "calibrating...");
        opmode.telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!opmode.isStopRequested() && !imu.isGyroCalibrated()) {
            opmode.sleep(50);
            opmode.idle();
        }

        opmode.telemetry.addData("Mode", "waiting for start");
        opmode.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        opmode.telemetry.update();

    }

    public void setThread(Runnable run, Runnable run2){
        turretTurn = new Thread(run);
        liftThread = new Thread(run2);
    }


    public void startOuttakeThread(double pwr, double time){
        this.pwr = pwr;
        this.time = time;
        outtakeThread.start();
    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public void outtake (double time, double pwr){
        ElapsedTime timer = new ElapsedTime();

        while(timer.seconds() < time){
            roller1.setPower(pwr);
            roller2.setPower(pwr);
        }

        roller1.setPower(0);
        roller2.setPower(0);
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

    public void drop(){



    }



    public void turnPID(double pwr, double angle, double p, double i, double d, double timeout) { //This is the good PID method use this one
        double initPos = getAngle();
        ElapsedTime runtime = new ElapsedTime();
        double integral = 0;
        while(Math.abs(angle - initPos) >= 1 && runtime.seconds() < timeout){
            double prevError = angle - initPos;
            double proportional = p * prevError;
            double prevTime = runtime.seconds();
            integral += i * (prevError * (runtime.seconds() - prevTime));
            double derivative = d * ((angle - getAngle() - prevError) / (runtime.seconds() - prevTime));

            fL.setPower((proportional + integral + derivative) * pwr);
            fR.setPower(-(proportional + integral + derivative) * pwr);
            bL.setPower((proportional + integral + derivative) * pwr);
            bR.setPower((proportional + integral + derivative) * pwr);


            opmode.telemetry.addData("angle: ", getAngle());
            opmode.telemetry.update();

            initPos = getAngle();
        }

        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }

    public double getAvgEncoder() {

        double div = 4;
        double avgAdd = bL.getCurrentPosition() + bR.getCurrentPosition() + fL.getCurrentPosition() + fR.getCurrentPosition();
        double avg;

        if (bL.getCurrentPosition() == 0) {
            div--;
        }

        if (bR.getCurrentPosition() == 0) {
            div--;
        }
        if (fL.getCurrentPosition() == 0) {
            div--;
        }

        if (fR.getCurrentPosition() == 0) {
            div--;
        }

        if (div == 0) {
            avg = 0;
        } else {
            avg = avgAdd / div;
        }

        return -avg;
    }

    public void straight(double pwr, double RhAdj, double LhAdj) {

        double max = Math.max(Math.abs(pwr + LhAdj), Math.abs(pwr + RhAdj));
        double leftPwr = pwr + LhAdj;
        double rightPwr = pwr + RhAdj;

        if (max > 1) {
            leftPwr /= max;
            rightPwr /= max;
        }

        fL.setPower(-leftPwr);
        fR.setPower(-rightPwr);
        bL.setPower(-leftPwr);
        bR.setPower(-rightPwr);
    }

    public void stopAll() {

        double pwr = 0;
        fL.setPower(pwr);
        fR.setPower(pwr);
        bL.setPower(pwr);
        bR.setPower(pwr);

    }

    public void goStraightPID(double distance, double kP, double kI, double kD, double timeout, double max) {
        ElapsedTime timer = new ElapsedTime();
        resetEncoders();
        timer.startTime();

        double oldGyro = 0;
        double power;
        double error = 0;
        double currTime = timer.milliseconds();
        double LhAdjust = 0;
        double RhAdjust = 0;
        double integral = 0;
        double derivative;
        double proportional;
        double oldTime = currTime;


        while (Math.abs(distance) > Math.abs(getAvgEncoder()) && !opmode.isStopRequested()) {
            currTime = timer.milliseconds();

            proportional = (distance - getAvgEncoder()) * kP;
            integral += (distance - getAvgEncoder()) * (currTime - oldTime) * kI;
            derivative = getAngle() - oldGyro * kD;
            power = integral + proportional + derivative;

            error = getAngle();

            RhAdjust = -(error * .028);
            LhAdjust = (error * .035);

            if(power < 0.15 && distance > 0){
                power = 0.2;
            }
            if(power > -0.15 && distance < 0){
                power = -0.2;
            }

            if(Math.abs(power) > Math.abs(max)){
                power = max;
            }
            oldTime = currTime;
            straight(power, RhAdjust, LhAdjust);

            opmode.telemetry.addData("Avg Encoder Val", getAvgEncoder());
            opmode.telemetry.addData("Gyro Error", error);
            opmode.telemetry.addData("Amount left", (distance - getAvgEncoder()));
            opmode.telemetry.addData("Forward power", power);
            opmode.telemetry.addData("Proportional", proportional);
            opmode.telemetry.addData("Integral", integral);
            opmode.telemetry.addData("Derivatve", derivative);
            opmode.telemetry.addData("Left power: ", LhAdjust);
            opmode.telemetry.addData("Right power: ", RhAdjust);
            opmode.telemetry.update();

            if (currTime > timeout) {
                break;
            }

            oldGyro = getAngle();
        }


        stopAll();
    }

    public void turretPID(double pwr, double angle, double kp, double ki, double kd, double timeout){
        double prev = getTurretAngle(turret.getCurrentPosition());
        double current = getTurretAngle(turret.getCurrentPosition());
        ElapsedTime runtime = new ElapsedTime();
        double p;
        double i = 0;
        double d;
        double preTime = 0;

        while(Math.abs(current - angle) > 1 && runtime.seconds() < timeout){
            current = getTurretAngle(turret.getCurrentPosition());
            double currTime = runtime.seconds();

            p = (current - angle) * kp;
            i += (current - prev) * (currTime - preTime) * ki;
            d = ((current - prev)/(currTime - preTime)) * kd;

            pwr = p + i + d;

            turret.setPower(pwr);
            preTime = currTime;
            prev = current;

        }

        turret.setPower(0);
    }

    public double getTurretAngle(double ticks){
        double totalTicks = 120;
        return ticks/totalTicks * 360;
    }

    public void resetEncoders() {

        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();

        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opmode.idle();
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opmode.idle();
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opmode.idle();
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opmode.idle();

    }

}

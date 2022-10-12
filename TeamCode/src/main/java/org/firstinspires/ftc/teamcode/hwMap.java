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
    public DcMotor fL;
    public DcMotor fR;
    public DcMotor bR;
    public DcMotor bL;
    public DcMotor turret;
    public DcMotor lift;

    public CRServo roller1;
    public CRServo roller2;

    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    public Orientation startPos = null;
    public double lastDegrees;
    public double globalAngle;
    public double referenceAngle;

    LinearOpMode opmode;
    ElapsedTime runtime = new ElapsedTime();

    public hwMap(LinearOpMode opmode) {
        this.opmode = opmode;
        fL = opmode.hardwareMap.get(DcMotor.class, "fL");
        fR = opmode.hardwareMap.get(DcMotor.class, "fR");
        bL = opmode.hardwareMap.get(DcMotor.class, "bL");
        bR = opmode.hardwareMap.get(DcMotor.class, "bR");

        turret = opmode.hardwareMap.get(DcMotor.class, "turret");
        lift = opmode.hardwareMap.get(DcMotor.class, "lift");

        roller1 = opmode.hardwareMap.get(CRServo.class, "roller1");
        roller2 = opmode.hardwareMap.get(CRServo.class, "roller2");

        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = opmode.hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
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

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
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

    public void turretPID(double pwr, double angle, double p, double i, double d, double timeout){
        double initPos = 0;
        double initTurretPos = 0;
        ElapsedTime runtime = new ElapsedTime();
        double integral = 0;
        while(Math.abs(angle - initPos) >= 1 && runtime.seconds() < timeout){
            double currTurretPos = turret.getCurrentPosition();
            double prevError = angle - initPos;
            double proportional = p * prevError;
            double prevTime = runtime.seconds();
            integral += i * (prevError * (runtime.seconds() - prevTime));
            double derivative = d * ((angle - getTurretAngle(currTurretPos - initTurretPos) - prevError) / (runtime.seconds() - prevTime));

            turret.setPower(proportional + integral + derivative);

            initPos = getTurretAngle(Math.abs(currTurretPos - initTurretPos));
        }
        turret.setPower(0);
    }

    public void encoderMove(double speed, double target){
        ElapsedTime runtime = new ElapsedTime();
        double initPos = fR.getCurrentPosition();

        while(opmode.opModeIsActive() && Math.abs(fR.getCurrentPosition() - initPos) < target){
            fL.setPower(speed);
            fR.setPower(speed);
            bL.setPower(speed);
            bR.setPower(speed);
        }

        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }

    public double getTurretAngle(double ticks){
        double turretRadius = (9 + 7 / 8.0) / 2;
        double ticksToInches = 0.0; //find

        return (ticks * ticksToInches) / turretRadius * 360;
    }
}

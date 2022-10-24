package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "TeleOp", name = "PPTeleOp")
public class PPTeleOp extends LinearOpMode {
    hwMap hw;
    @Override
    public void runOpMode() {

        hw = new hwMap(this );
        waitForStart();
        while(opModeIsActive()){
            trigMecanum();

            if(gamepad1.right_trigger > 0.1){
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
            }

        }
    }

    public void trigMecanum() {

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

        hw.fL.setPower(-v1);
        hw.fR.setPower(-v2);
        hw.bL.setPower(v3);// * .79);
        hw.bR.setPower(v4);// * .79);
    }

}

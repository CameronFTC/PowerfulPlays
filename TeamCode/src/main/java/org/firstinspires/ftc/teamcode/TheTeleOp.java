package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "TeleOp", name = "TheTeleOp")
public class TheTeleOp extends LinearOpMode {
    hwMap hw;
    @Override
    public void runOpMode() throws InterruptedException {
        hw = new hwMap(this);
        waitForStart();
        while(opModeIsActive()){
            drive();
//            if (gamepad2.a) {
//                hw.arm1.setPower(1);
//                hw.arm2.setPower(-1);
//            }
//            else if (gamepad2.b) {
//                hw.arm1.setPower(-.4);
//                hw.arm2.setPower(.4);
//            }
//            else{
//                hw.arm1.setPower(0);
//                hw.arm2.setPower(0);
//            }
            if(gamepad2.right_stick_y > 0.1){
                hw.lift.setPower(gamepad2.right_stick_y);
                hw.lift2.setPower(gamepad2.right_stick_y);
            }
            else{
                hw.lift.setPower(gamepad2.right_stick_y * 0.6);
                hw.lift2.setPower(gamepad2.right_stick_y * 0.6);
            }
            //hw.roller1.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
            //.roller2.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        }
    }

    public void drive(){
        hw.fL.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x * 0.75));
        hw.fR.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.right_stick_x * 0.75));
        hw.bL.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.right_stick_x * 0.75));
        hw.bR.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.right_stick_x * 0.75));
    }
}

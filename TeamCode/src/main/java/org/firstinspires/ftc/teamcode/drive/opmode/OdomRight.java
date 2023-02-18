package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class OdomRight extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-36, -63, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-33, -18), Math.toRadians(-45))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(-33, -18, Math.toRadians(-45)))
                .splineTo(new Vector2d(-54, -10), Math.toRadians(0))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectory(traj1);

        sleep(1000);

        drive.followTrajectory(traj2);
    }
}

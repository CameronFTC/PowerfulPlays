package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive", name = "OdomLeft")
public class OdomRight extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-26, -63, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-28, -18), Math.toRadians(142))
                //.back(3)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(-54, -13), Math.toRadians(180))
                .addTemporalMarker(2, () -> {
                    drive.armStack();
                })
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(-54, -13), Math.toRadians(180))
                .addTemporalMarker(2, () -> {
                    drive.lift(-800, 0.6);
                    drive.armStack();
                })
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(-25, -18, Math.toRadians(142)))
                .addTemporalMarker(2, () -> {
                    drive.lift(800, 0.6);
                    drive.armStack();
                })
                .build();

        //hwMap hw = new hwMap(this);

        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectory(traj1);

        sleep(300);
        //hw.pickUpThread.start();
        //Cycle 1
        drive.followTrajectory(traj2);


        sleep(300);

        drive.followTrajectory(traj3);

        sleep(300);

        //Cycle 2
        drive.followTrajectory(traj4);

        sleep(300);

        drive.followTrajectory(traj3);

        sleep(300);

        //Cycle 3
        drive.followTrajectory(traj4);

        sleep(300);

        drive.followTrajectory(traj3);

        sleep(300);

        //Cycle 4
        drive.followTrajectory(traj4);

        sleep(300);

        drive.followTrajectory(traj3);

        sleep(300);

        //Cycle 5
        drive.followTrajectory(traj4);

        sleep(300);

        drive.followTrajectory(traj3);

        sleep(300);
    }
}

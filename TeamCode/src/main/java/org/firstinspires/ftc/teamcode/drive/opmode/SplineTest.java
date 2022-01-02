package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(48, -48, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .forward(48)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(0,0),SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        drive.followTrajectory(traj1);
        sleep(1000);
        drive.followTrajectory(traj2);



    }
}

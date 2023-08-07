package org.firstinspires.ftc.teamcode.ChasisTank.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

public class TrayectoryAuto {
    public static Trajectory Samaniego(SampleTankDrive drive) {
        Pose2d firstPose = new Pose2d(-38,-61, Math.toRadians(270));

        drive.setPoseEstimate(firstPose);
        return drive.trajectoryBuilder(firstPose)
                .forward(40)
                .build();
    }

    public Trajectory leftCone(SampleTankDrive drive) {

        return drive.trajectoryBuilder(new Pose2d(-38, 17, Math.toRadians(180)))
                .strafeLeft(30)
                .build();
    }

    public Trajectory goCone(SampleTankDrive drive) {

        return drive.trajectoryBuilder(new Pose2d(-38, -3, Math.toRadians(180)))
                .forward(12.5)
                .build();
    }

    public Trajectory goToJunction(SampleTankDrive drive) {

        return drive.trajectoryBuilder(new Pose2d(-43, -5, Math.toRadians(180)))
                .strafeTo(new Vector2d(-45, 14))
                .build();
    }

    public Trajectory rigthStat(SampleTankDrive drive){
        return drive.trajectoryBuilder(new Pose2d(-45, -30, Math.toRadians(180)))
                .strafeRight(24)
                .build();
    }

    public Trajectory forwardStat(SampleTankDrive drive){
        return drive.trajectoryBuilder(new Pose2d(-39, -8, Math.toRadians(180)))
                .forward(30)
                .build();
    }

    public Trajectory backStat(SampleTankDrive drive){
        return drive.trajectoryBuilder(new Pose2d(-39, -15, Math.toRadians(180)))
                .back(26)
                .build();
    }

}

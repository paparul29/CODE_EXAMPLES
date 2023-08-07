package org.firstinspires.ftc.teamcode.ChasisTank.commands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ChasisTank.subsystems.TankSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

public class TrajectoryFollowerCommand extends CommandBase {

    private final TankSubsystem drive;
    private final Trajectory trajectory;

    public TrajectoryFollowerCommand(TankSubsystem drive, Trajectory trajectory) {
        this.drive = drive;
        this.trajectory = trajectory;

        addRequirements(drive);


    }

    @Override
    public void initialize() {
        drive.followTrajectory(trajectory);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }



}
package org.firstinspires.ftc.teamcode.ChasisTank.Auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ChasisTank.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.ChasisTank.subsystems.TankSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

public class AutoCommand extends SequentialCommandGroup{

    TrayectoryAuto trayectoryAuto = new TrayectoryAuto();

    public AutoCommand(TankSubsystem drive) {



        addCommands(
                //new TrajectoryFollowerCommand(drive, TrayectoryAuto.Samaniego(drive.getDrive()))
                );

    }

}

package org.firstinspires.ftc.teamcode.ChasisTank.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ChasisTank.subsystems.TankSubsystem;

import java.util.function.DoubleSupplier;

public class TankDriveCommand extends CommandBase {

    private final TankSubsystem drive;
    private final DoubleSupplier leftY;
    private final DoubleSupplier rightX;

    public TankDriveCommand(TankSubsystem drive, DoubleSupplier leftY, DoubleSupplier rightX) {
        this.drive = drive;
        this.leftY = leftY;
        this.rightX = rightX;

        addRequirements(drive);
    }

    @Override
    public void execute(){
        drive.drive(leftY.getAsDouble(), rightX.getAsDouble());
    }


}
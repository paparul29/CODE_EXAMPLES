package org.firstinspires.ftc.teamcode.ChasisTank.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ChasisTank.subsystems.Brazo;

public class BrazoRunn extends CommandBase {

    Brazo brazo;
    int setpoint;

    public BrazoRunn(Brazo brazo, int setpoint){
        this.brazo =  brazo;
        this.setpoint = setpoint;

        addRequirements(brazo);
    }

    @Override
    public void execute (){
    }



}

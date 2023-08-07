package org.firstinspires.ftc.teamcode.ChasisTank.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Roller extends SubsystemBase {
   /* DcMotorEx intakemotor;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Roller(Telemetry telemetry, HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        intakemotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakemotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakemotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakemotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void get(){
        intakemotor.setPower(.9);
        intakemotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void stop(){
        intakemotor.setPower(0);
    }
    public void out(){
        intakemotor.setPower(.9);
        intakemotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }*/
}

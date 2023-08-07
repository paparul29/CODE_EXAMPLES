package org.firstinspires.ftc.teamcode.ChasisTank.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BrazoServo extends SubsystemBase {

    ServoEx servoSube;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public BrazoServo(Telemetry telemetry, HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        servoSube = new SimpleServo(hardwareMap, "servoUp",0,180);
    }
    public void sube(){
        servoSube.turnToAngle(0);
    }
    public void baja(){
        servoSube.turnToAngle(180);
    }


}

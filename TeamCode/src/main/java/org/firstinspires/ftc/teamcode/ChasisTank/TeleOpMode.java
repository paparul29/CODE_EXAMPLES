package org.firstinspires.ftc.teamcode.ChasisTank;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ChasisTank.commands.TankDriveCommand;
import org.firstinspires.ftc.teamcode.ChasisTank.subsystems.Brazo;
import org.firstinspires.ftc.teamcode.ChasisTank.subsystems.BrazoServo;
import org.firstinspires.ftc.teamcode.ChasisTank.subsystems.Intake1S;
import org.firstinspires.ftc.teamcode.ChasisTank.subsystems.Intake2S;
import org.firstinspires.ftc.teamcode.ChasisTank.subsystems.TankSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@TeleOp
public class TeleOpMode extends CommandOpMode {

    @Override
    public void initialize() {

        GamepadEx gamepadDriver = new GamepadEx(gamepad1);
        GamepadEx gamepadsystem = new GamepadEx(gamepad2);

        SampleTankDrive sampleTankDrive = new SampleTankDrive(hardwareMap);
        TankSubsystem driveSystem = new TankSubsystem(sampleTankDrive);
        Intake2S pinza = new Intake2S(telemetry, hardwareMap);
        Brazo brazo = new Brazo(telemetry, hardwareMap);
        //Roller roller = new Roller(telemetry, hardwareMap);
        //Intake1S claw = new Intake1S(telemetry, hardwareMap);
        //BrazoServo brazoUp = new BrazoServo(telemetry, hardwareMap);


        driveSystem.setDefaultCommand(new TankDriveCommand(
                driveSystem, () -> -gamepadDriver.getLeftY(), gamepadDriver::getRightX
        ));

        schedule(new RunCommand(() -> {
            driveSystem.update();
            telemetry.addData("Heading", driveSystem.getPoseEstimate().getHeading());
            telemetry.update();
        }));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(pinza::open));
        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(pinza::close));


        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.Y)
                .whenPressed(() -> brazo.upArm(-5400, .5));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.X)
                .whenPressed(() -> brazo.upArm(-2800, .5));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.A)
                .whenPressed(() -> brazo.setPosition(0, .5));

/*
        new GamepadButton(new GamepadEx(gamepad1),GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(claw::deja));
        new GamepadButton(new GamepadEx(gamepad1),GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(claw::agarra));


 */


        //new GamepadButton(new GamepadEx(gamepad2),GamepadKeys.Button.B)
        //.whenPressed(() -> brazo.upArm(455,.5));

        //new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_RIGHT)
        //      .whenPressed(new InstantCommand(roller::get))
        //    .whenReleased(new InstantCommand(roller::stop));

        //new GamepadButton(new GamepadEx(gamepad1),GamepadKeys.Button.LEFT_BUMPER)
        //      .whenPressed(new InstantCommand(roller::out))
        //    .whenReleased(new InstantCommand(roller::stop))

    }
}


package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

public class TestPoleSubsystem extends CommandOpMode {

    @Override
    public void initialize() {
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        PoleSubsystem pole = new PoleSubsystem(hardwareMap, "pole01");
    }

    @Override
    public void run() {
        //Loop:
        CommandScheduler.getInstance().run();

        telemetry.addData("Initialization . . . ", "started");
        telemetry.update();

        if (gamepad1.a) {
            schedule(
//                    new pole.PoleUp(robot)
            );
        }
    }
}
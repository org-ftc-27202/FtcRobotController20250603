package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.attachments.BotPole;
import org.firstinspires.ftc.teamcode.subsystems.PoleSubsystem;

import java.util.function.BooleanSupplier;

@TeleOp(name = "AA Gamepad Sample", group = "Tests")

public class GamepadSample extends LinearOpMode {
    private GamepadEx driverOp;
    private PoleSubsystem pole;

    @Override
    public void runOpMode() throws InterruptedException {
        driverOp = new GamepadEx(gamepad1);
        pole = new PoleSubsystem(hardwareMap, "pole01");

        waitForStart();

        while (opModeIsActive()) {

            if (driverOp.isDown(GamepadKeys.Button.A)) {
                pole.MoveToStraightUpPosition();
            }

            driverOp.readButtons();
            idle();
        }
    }
}
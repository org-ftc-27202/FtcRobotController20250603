package org.firstinspires.ftc.teamcode.drivers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.attachments.BotPole;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

@TeleOp(name = "AS5600 Test", group = "Tests")
public class TestAS5600 extends OpMode {
    final short MAGNETIC_ENCODER_01_RAW_ANGLE_ZERO_POSITION = 735;

    AS5600 magneticEncoder;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        AS5600 magneticEncoder = hardwareMap.get(AS5600.class, "magneticEncoder01");
        try {
            magneticEncoder.setZeroPosition(MAGNETIC_ENCODER_01_RAW_ANGLE_ZERO_POSITION);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        telemetry.addData("Manufacturer", magneticEncoder.getManufacturer());
        telemetry.addData("Device Name", magneticEncoder.getDeviceName());
        telemetry.addData("Status Raw", magneticEncoder.getStatusRaw());
        telemetry.addData("Is Magnet Detected?", magneticEncoder.isMagnetDetected());
        telemetry.addData("Is Magnet Too Weak?", magneticEncoder.isMagnetTooWeak());
        telemetry.addData("Is Magnet Too Strong?", magneticEncoder.isMagnetTooStrong());
        telemetry.addData("Angle", "%.2f", magneticEncoder.getAngle());
        telemetry.addData("Raw Angle", "%d", magneticEncoder.getRawAngle());

        telemetry.update();
    }
}
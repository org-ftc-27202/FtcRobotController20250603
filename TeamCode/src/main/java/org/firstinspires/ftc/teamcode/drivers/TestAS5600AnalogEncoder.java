package org.firstinspires.ftc.teamcode.drivers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "AS5600 Analog Test", group = "Tests")
public class TestAS5600AnalogEncoder extends OpMode {

    AnalogEncoder encoder;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        encoder = new AnalogEncoder("analog", hardwareMap);
        encoder.setInverted(true);
        encoder.setPositionOffset(0.0);
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

//            telemetry.addData("Manufacturer", magneticEncoder.getManufacturer());
//            telemetry.addData("Device Name", magneticEncoder.getDeviceName());
//            telemetry.addData("Status Raw", magneticEncoder.getStatusRaw());
//            telemetry.addData("Is Magnet Detected?", magneticEncoder.isMagnetDetected());
//            telemetry.addData("Is Magnet Too Weak?", magneticEncoder.isMagnetTooWeak());
//            telemetry.addData("Is Magnet Too Strong?", magneticEncoder.isMagnetTooStrong());
//            telemetry.addData("Angle", "%.2f", magneticEncoder.getAngle());
//            telemetry.addData("Raw Angle", "%d", magneticEncoder.getRawAngle());

        telemetry.addData("angle", "%f", encoder.getAngle());

        telemetry.update();
    }
}
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

@TeleOp(name = "AS5600 Test Pole", group = "Tests")
public class TestAS5600_Pole extends OpMode {
//    final short MAGNETIC_ENCODER_01_RAW_ANGLE_ZERO_POSITION = 735;
    private List<Action> runningActions = new ArrayList<>();
    private HashSet<Action> HashSetRunningActions = new HashSet<>();
    private BotPole pole;
//        AS5600 magneticEncoder = hardwareMap.get(AS5600.class, "magneticEncoder01");
//        magneticEncoder.setZeroPosition(MAGNETIC_ENCODER_01_RAW_ANGLE_ZERO_POSITION);
//        BotPole pole = new BotPole(hardwareMap, magneticEncoder);
    boolean flagActionCompleted = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pole = new BotPole(hardwareMap);
    }

    @Override
    public void start() {
        pole.MoveToStartingPosition();
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

        if (gamepad1.a) {
            pole.setPoleUpTarget();
//            HashSetRunningActions.add(new SequentialAction(
//                    pole.poleUp()
//            ));
        }

        // update running actions
//        List<Action> newActions = new ArrayList<>();
//        HashSet<Action> newActions = new HashSet<>();
//        for (Action action : HashSetrunningActions) {
//            action.preview(packet.fieldOverlay());
//            if (!action.run(packet)) {
//                newActions.add(action);
//            }
//        }
//        HashSetrunningActions = newActions;

//        HashSet<Action> newActions = new HashSet<>();
//        for (Action action : HashSetRunningActions) {
//            action.preview(packet.fieldOverlay());
//            if (action.run(packet)) {
//                newActions.add(action);
//            }
//        }
//        HashSetRunningActions = newActions;

        pole.ActionForPole();

        telemetry.addData("Running Actions Count", "%d", HashSetRunningActions.size());
//        telemetry.addData("New Actions Count", "%d", newActions.size());
        telemetry.addData("Pole Angle", "%.2f", pole.getAngle());
        telemetry.addData("called Counter", "%d", pole.getCalledCounter());
        telemetry.addData("run True Counter", "%d", pole.getRunTrueCounter());
        telemetry.addData("run False Counter", "%d", pole.getRunFalseCounter());

//        telemetry.addData("newActions Count", "%d", newActions.size());
        telemetry.update();
    }
}
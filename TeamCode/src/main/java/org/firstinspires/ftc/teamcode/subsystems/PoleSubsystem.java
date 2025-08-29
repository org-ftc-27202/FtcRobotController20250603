package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivers.AS5600;

public class PoleSubsystem extends SubsystemBase {
    private final short MAGNETIC_ENCODER_01_RAW_ANGLE_ZERO_POSITION = 735;
    private final float START_POSITION = 0;
    private final float UP_POSITION = 180;
    private DcMotorEx poleMotor;
    private AS5600 magneticEncoder;

    Telemetry telemetry;

    public PoleSubsystem(HardwareMap hardwareMap, final String deviceName) {
        poleMotor = hardwareMap.get(DcMotorEx.class, deviceName);
        poleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        poleMotor.setVelocity(20);
        poleMotor.setPower(0);

        magneticEncoder = hardwareMap.get(AS5600.class, "magneticEncoder01");
        try {
            magneticEncoder.setZeroPosition(MAGNETIC_ENCODER_01_RAW_ANGLE_ZERO_POSITION);
        }
        catch (InterruptedException e) {
            telemetry.addData("%s","Error: not able to reset magnetic encoder to zero position.");
        } ;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    public void MoveToStraightUpPosition() {
        float pos = 0;
        boolean flagAtPosition = false;
        float posTolerance = 0.5F;

        poleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        poleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pos = magneticEncoder.getAngle();
        flagAtPosition = (pos >= (UP_POSITION - posTolerance) && pos <= (UP_POSITION + posTolerance));
        while (!flagAtPosition) {
            telemetry.addData("%s","Inside MoveUp");
            telemetry.addData("Motor Current Position", "%d", poleMotor.getCurrentPosition());
            telemetry.addData("Encoder Position", "%f", pos);
            telemetry.update();

            poleMotor.setPower(0.25);
            pos = magneticEncoder.getAngle();
            flagAtPosition = (pos >= (UP_POSITION - posTolerance) && pos <= (UP_POSITION + posTolerance));
        }
        poleMotor.setPower(0.0); // stop the motor
        poleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public static class TestPoleSubsystem {
    }
}

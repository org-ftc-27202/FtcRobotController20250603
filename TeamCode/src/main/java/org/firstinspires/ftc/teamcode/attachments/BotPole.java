package org.firstinspires.ftc.teamcode.attachments;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivers.AS5600;

public class BotPole {
    public enum Direction {
        CLOCKWISE, COUNTERCLOCKWISE
    }
    final short MAGNETIC_ENCODER_01_RAW_ANGLE_ZERO_POSITION = 735;
    final float TOLERANCE_DEGREE = 0.5F;
    final float START_POSITION_DEGREE = 45;
    final float UP_POSITION_DEGREE = 180;
    private DcMotorEx poleMotor;
    private AS5600 magneticEncoder;
    private boolean flagOnTargetPosition = true;
    private int calledCounter = 0;
    private int runTrueCounter = 0;
    private int runFalseCounter = 0;
    private float TargetPosition = START_POSITION_DEGREE;
    Direction MoveDirection = Direction.CLOCKWISE;

    public BotPole(HardwareMap hardwareMap) {
        poleMotor = hardwareMap.get(DcMotorEx.class, "pole01");
        poleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        poleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        poleMotor.setVelocity(20);
        poleMotor.setPower(0);
        magneticEncoder = hardwareMap.get(AS5600.class, "magneticEncoder01");
        try {
            magneticEncoder.setZeroPosition(MAGNETIC_ENCODER_01_RAW_ANGLE_ZERO_POSITION);
        }
        catch (InterruptedException e) {
            poleMotor.setPower(0);
        } ;
    }

    public boolean OnTargetPosition() {
        return flagOnTargetPosition;
    }
    public int getCalledCounter() {
        return calledCounter;
    }
    public int getRunTrueCounter() { return runTrueCounter;}
    public int getRunFalseCounter() {
        return runFalseCounter;
    }
    public float getAngle() {
        return magneticEncoder.getAngle();
    }
    public void MoveToStartingPosition() {
        if (magneticEncoder.getAngle() >= TargetPosition) {
            MoveDirection = Direction.COUNTERCLOCKWISE;
        }
        else {
            MoveDirection = Direction.CLOCKWISE;
        }
    }
    public void setPoleUpTarget() {
        TargetPosition = UP_POSITION_DEGREE;
        MoveDirection = Direction.CLOCKWISE;
    }
    public void ActionForPole() {
        float pos = magneticEncoder.getAngle();
        if (MoveDirection == Direction.CLOCKWISE && pos >= TargetPosition) {
            poleMotor.setPower(0);
        }
        else if (MoveDirection == Direction.COUNTERCLOCKWISE && pos <= TargetPosition) {
            poleMotor.setPower(0);
        }
        else {
            if (MoveDirection == Direction.CLOCKWISE) {
                poleMotor.setPower(0.05 * -1.0);
            }
            else if (MoveDirection == Direction.COUNTERCLOCKWISE) {
                poleMotor.setPower(0.05);
            }
        }
    }
    public class poleUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            float pos = 0;

            if (!initialized) {
                initialized = true;
            }

            ++calledCounter;

            pos = magneticEncoder.getAngle();
            if (pos >= (TargetPosition - TOLERANCE_DEGREE) && pos <= (TargetPosition + TOLERANCE_DEGREE)) {
                poleMotor.setPower(0);
                flagOnTargetPosition = true;
                ++runTrueCounter;
                return true;
            } else {  // target position is not reached yet
                poleMotor.setPower(0.05);
                flagOnTargetPosition = false;
                ++runFalseCounter;
                return false;
            }
        }
    }


    public Action poleUp() {
        return new poleUp();
    }
}

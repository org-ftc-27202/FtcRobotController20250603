package org.firstinspires.ftc.teamcode.drivers;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

/**
 * A driver for the AMS AS5600 magnetic rotary encoder for FTC.
 */

// Ignore access and unused warnings
// Both driver classes cannot register the sensor at the same time. One driver should have the
// sensor registered, and the other should be commented out
@SuppressWarnings({"WeakerAccess", "unused"})
@I2cDeviceType
@DeviceProperties(name = "AMS AS5600", description = "AMS AS5600 magnetic rotary encoder", xmlTag = "AS5600")

public class AS5600 extends I2cDeviceSynchDevice<I2cDeviceSynch> {
//    private final ReadWriteLock readWriteLock = new ReentrantReadWriteLock();
//    private final Lock readLock = readWriteLock.readLock();
//    private final Lock writeLock = readWriteLock.writeLock();

    @Override
    public Manufacturer getManufacturer() {return Manufacturer.AMS;}

    @Override
    public String getDeviceName() {return "AS5600 Magnetic Rotary Encoder";}

    // Constructor and Initialization
    // Default I2C address of the AS5600
    // https://look.ams-osram.com/m/7059eac7531a86fd/original/AS5600-DS000365.pdf
    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x36);
    public int maxAngle = 360;

    public AS5600(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        this.deviceClient.engage();
    }

    // Registers and Config Settings
    public enum Register
    {
        FIRST(0),
        REGISTER_ZMCO(0x00),
        REGISTER_ZPOS_HIGH(0x01),   // ZPOS = start position
        REGISTER_ZPOS_LOW(0x02),    // ZPOS = start position
        REGISTER_MPOS_HIGH(0x03),   // MPOS = stop position
        REGISTER_MPOS_LOW(0x04),   // MPOS = stop position
        REGISTER_MANG_HIGH(0x05),   // MANG = maximum angle
        REGISTER_MANG_LOW(0x06),   // MANG = maximum angle
        REGISTER_CONF_HIGH(0x07),
        REGISTER_CONF_LOW(0x08),
        REGISTER_RAW_ANGLE_HIGH( 0x0C),
        REGISTER_RAW_ANGLE_LOW( 0x0D),
        REGISTER_ANGLE_HIGH(0x0E),
        REGISTER_ANGLE_LOW(0x0F),
        REGISTER_STATUS(0x0B),
        REGISTER_AUTOMATIC_GAIN_CONTROL(0x1A),
        REGISTER_MAGNITUDE_HIGH(0x1B),
        REGISTER_MAGNITUDE_LOW(0x1C),
        REGISTER_BURN(0xFF),
        LAST(REGISTER_BURN.bVal);

        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return isMagnetDetected();
    }

    protected void writeShort(final Register reg, short value)
    {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    // Raw Register Reads
    public short getStatusRaw() {
        // bit 3: MH AGC minimum gain overflow, magnet too strong
        // bit 4: ML AGC maximum gain overflow, magnet too weak
        // bit 5: MD Magnet was detected

        return readShort(Register.REGISTER_STATUS);
    }

    // Checks if a magnet is detected by the sensor.  @return true if magnet is detected, false otherwise.
    public boolean isMagnetDetected() {
        return (getStatusRaw() & 0x2000) == 0x2000;
    }

    public boolean isMagnetTooWeak() {
        return (getStatusRaw() & 0x1000) == 0x1000;
    }

    public boolean isMagnetTooStrong() {
        return (getStatusRaw() & 0x0800) == 0x0800;
    }

    // Reads the scaled angle from the AS5600 (0-360).
    public void setZeroPosition(short desiredRawAngleAtZeroPosition) throws InterruptedException {
        short rawAngle = desiredRawAngleAtZeroPosition;

        writeShort(Register.REGISTER_ZPOS_HIGH, rawAngle);
        Thread.sleep(10);

        // set the stop position to the desired raw angle - 1
        if (rawAngle == 0) {
            writeShort(Register.REGISTER_MPOS_HIGH, (short) 4095);
        }
        else {
            rawAngle -= 1;
            writeShort(Register.REGISTER_MPOS_HIGH, rawAngle);
        }
        Thread.sleep(10);
    }

    public short getRawAngle() {
        return readShort(Register.REGISTER_RAW_ANGLE_HIGH);
    }

    public float getAngle() {
        int angleHigh = readShort(Register.REGISTER_ANGLE_HIGH) & 0x0FFF;
        return (float) (angleHigh / 4096.0 * maxAngle);
    }
//    // Reads the raw angle from the AS5600 (0-4095).
//    public int getRawAngle() {
//        readLock.lock();
//        try {
//            byte[] buffer = deviceClient.read(REGISTER_RAW_ANGLE_HIGH, 2);
//            if (buffer != null && buffer.length == 2) {
//                int highByte = buffer[0] & 0xFF;
//                int lowByte = buffer[1] & 0xFF;
//                return (highByte << 8) | lowByte;
//            }
//            return 0; // Return 0 on error.  Consider throwing an exception.
//        } finally {
//            readLock.unlock();
//        }
//    }
//
//    // Reads the compensated angle from the AS5600 (0-4095).
//    public int getAngle() {
//        readLock.lock();
//        try {
//            byte[] buffer = deviceClient.read(REGISTER_ANGLE_HIGH, 2);
//            if (buffer != null && buffer.length == 2) {
//                int highByte = buffer[0] & 0xFF;
//                int lowByte = buffer[1] & 0xFF;
//                return (highByte << 8) | lowByte;
//            }
//            return 0; // Return 0 on error. Consider throwing an exception.
//        } finally {
//            readLock.unlock();
//        }
//    }
//
//    /**
//     * Checks if the last read operation had a successful transfer.
//     * @return true if last transfer was successful
//     */
//    public boolean isLastTransferSuccessful(){
//        readLock.lock();
//        try{
//            byte[] buffer = deviceClient.read(REGISTER_STATUS, 1);
//            if (buffer != null && buffer.length == 1) {
//                return (buffer[0] & 0x10) == 0; //check the 5th bit
//            }
//            return false;
//        } finally {
//            readLock.unlock();
//        }
//    }
//
//    /**
//     * Sets the start position (ZPOS) of the encoder.
//     * @param position The start position (0-4095).
//     */
//    public void setStartPosition(int position) {
//        writeLock.lock();
//        try {
//            //write start position
//            deviceClient.write(REGISTER_ZPOS_HIGH, (byte) (position >> 8));
//            deviceClient.write(REGISTER_ZPOS_LOW, (byte) (position & 0xFF));
//        } finally {
//            writeLock.unlock();
//        }
//    }
//
//    /**
//     * Gets the start position (ZPOS) of the encoder.
//     * @return start position.
//     */
//    public int getStartPosition() {
//        readLock.lock();
//        try {
//            byte[] buffer = deviceClient.read(REGISTER_ZPOS_HIGH, 2);
//            if (buffer != null && buffer.length == 2) {
//                int highByte = buffer[0] & 0xFF;
//                int lowByte = buffer[1] & 0xFF;
//                return (highByte << 8) | lowByte;
//            }
//            return 0;
//        } finally {
//            readLock.unlock();
//        }
//    }
//
//    /**
//     * Sets the stop position (MPOS) of the encoder.
//     * @param position The stop position (0-4095).
//     */
//    public void setStopPosition(int position) {
//        writeLock.lock();
//        try {
//            // Write the stop position
//            deviceClient.write(REGISTER_MPOS_HIGH, (byte) (position >> 8));
//            deviceClient.write(REGISTER_MPOS_LOW, (byte) (position & 0xFF));
//        } finally {
//            writeLock.unlock();
//        }
//    }
//
//    /**
//     * Gets the stop position (MPOS) of the encoder.
//     * @return the stop position.
//     */
//    public int getStopPosition(){
//        readLock.lock();
//        try{
//            byte[] buffer = deviceClient.read(REGISTER_MPOS_HIGH, 2);
//            if(buffer != null && buffer.length == 2){
//                int highByte = buffer[0] & 0xFF;
//                int lowByte = buffer[1] & 0xFF;
//                return (highByte << 8) | lowByte;
//            }
//            return 0;
//        } finally {
//            readLock.unlock();
//        }
//    }
//
//    /**
//     * Sets the maximum angle.
//     * @param angle
//     */
//    public void setMaximumAngle(int angle){
//        writeLock.lock();
//        try{
//            deviceClient.write(REGISTER_MANG_HIGH, (byte)(angle >> 8));
//            deviceClient.write(REGISTER_MANG_LOW, (byte)(angle & 0xFF));
//        } finally {
//            writeLock.unlock();
//        }
//    }
//
//    /**
//     * Gets the maximum angle.
//     * @return
//     */
//    public int getMaximumAngle(){
//        readLock.lock();
//        try{
//            byte[] buffer = deviceClient.read(REGISTER_MANG_HIGH, 2);
//            if(buffer != null && buffer.length == 2){
//                int highByte = buffer[0] & 0xFF;
//                int lowByte = buffer[1] & 0xFF;
//                return (highByte << 8) | lowByte;
//            }
//            return 0;
//        } finally{
//            readLock.unlock();
//        }
//    }
//
//    /**
//     * Sets the configuration register.  See the AS5600 datasheet for details
//     * on the individual bits.
//     * @param configuration
//     */
//    public void setConfiguration(int configuration) {
//        writeLock.lock();
//        try {
//            deviceClient.write(REGISTER_CONF_HIGH, (byte) (configuration >> 8));
//            deviceClient.write(REGISTER_CONF_LOW, (byte) (configuration & 0xFF));
//        } finally {
//            writeLock.unlock();
//        }
//    }
//
//    /**
//     * Gets the configuration register.
//     * @return
//     */
//    public int getConfiguration(){
//        readLock.lock();
//        try{
//            byte[] buffer = deviceClient.read(REGISTER_CONF_HIGH, 2);
//            if(buffer != null && buffer.length == 2){
//                int highByte = buffer[0] & 0xFF;
//                int lowByte = buffer[1] & 0xFF;
//                return (highByte << 8) | lowByte;
//            }
//            return 0;
//        } finally {
//            readLock.unlock();
//        }
//    }
}
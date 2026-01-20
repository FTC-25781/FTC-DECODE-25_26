package org.firstinspires.ftc.teamcode.layeredFinal.physical.colorSensorDriver;

import static com.qualcomm.robotcore.util.TypeConversion.byteArrayToInt;

import static java.lang.Thread.sleep;
// example:
// https://github.com/FIRST-Tech-Challenge/WikiSupport/blob/master/SampleOpModes/java/i2cExample/MCP9808.java
// https://github.com/goBILDA-Official/FtcRobotController-Add-Pinpoint/blob/goBILDA-Odometry-Driver/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/GoBildaPinpointDriver.java
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;

@I2cDeviceType
@DeviceProperties(
        name = "Color Sensor",
        xmlTag = "Voltage",
        description ="Voltage Color Sensor"
)

public class colorSensorDriver extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {
    public static final byte DEFAULT_ADDRESS = 0x29;

    // Enable register bits
    private static final byte ENABLE_AIEN  = (byte)0x10;
    private static final byte ENABLE_WEN   = (byte)0x08;
    private static final byte ENABLE_AEN   = (byte)0x02;
    private static final byte ENABLE_PON   = (byte)0x01;

    public double clear = 0.0;
    public double red = 0.0;
    public double green = 0.0;
    public double blue = 0.0;

    public int rclear = 0;
    public int rred = 0;
    public int rgreen = 0;
    public int rblue = 0;

    // sensor 3 values
    public int redCalBlackValue;
    public int greenCalBlackValue;
    public int blueCalBlackValue;

    public int redCalWhiteValue;
    public int greenCalWhiteValue;
    public int blueCalWhiteValue;

    // Register addresses
    private enum Register {
        REG_ENABLE  ((byte) 0x00), // Write: Enables states and interrupts
        REG_ATIME   ((byte) 0x01), // R/W: RGBC time
        REG_PON     ((byte) 0x01), // Power on - Writing 1 activates the internal oscillator, 0 disables it
        REG_RGBCEN  ((byte) 0x02),// RGBC Enable - Writing 1 actives the ADC, 0 disables it
        REG_WTIME   ((byte) 0x03), // R/W: Wait time
        REG_CONFIG  ((byte) 0x0D), // R/W: Configuration
        REG_CONTROL ((byte) 0x0F), // R/W: Control
        REG_STATUS  ((byte) 0x13), // R: Device Status
        REG_ID      ((byte) 0x12), // R: Device ID
        REG_CDATAL  ((byte) 0x14), // R: Clear channel data Low
        REG_CDATAH  ((byte) 0x15), // R: Clear channel data High
        REG_RDATAL  ((byte) 0x16), // R: Red channel data Low
        REG_RDATAH  ((byte) 0x17), // R: Red channel data High
        REG_GDATAL  ((byte) 0x18), // R: Green channel data High
        REG_GDATAH  ((byte) 0x19), // R: Green channel data Low
        REG_BDATAL  ((byte) 0x1A), // R: Blue channel data High
        REG_BDATAH  ((byte) 0x1B); // R: Blue channel data Low
        private final byte bVal;

        Register(byte bVal){
            this.bVal = bVal;
        }
    }

    public colorSensorDriver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_ADDRESS));
        super.registerArmingStateCallback(false);
    }

    public void tuneVals(
            int redCalBlackValue, int greenCalBlackValue,
            int blueCalBlackValue, int redCalWhiteValue,
            int greenCalWhiteValue, int blueCalWhiteValue
    ) {
        this.redCalBlackValue = redCalBlackValue;
        this.greenCalBlackValue = greenCalBlackValue;
        this.blueCalBlackValue = blueCalBlackValue;
        this.redCalWhiteValue = redCalWhiteValue;
        this.greenCalWhiteValue = greenCalWhiteValue;
        this.blueCalWhiteValue = blueCalWhiteValue;
    }

    @Override
    protected boolean doInitialize() {
        ((LynxI2cDeviceSynch)(deviceClient)).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);

        byte id = read8(Register.REG_ID.bVal);
        // Power on sequence: write PON, then AEN
        write8(Register.REG_ENABLE.bVal, ENABLE_PON);
        // sleep(3); // at least 2.4 ms
        write8(Register.REG_ENABLE.bVal, (byte)(ENABLE_PON | ENABLE_AEN));
        // Set integration time (e.g., 0xEB ~ 100 ms; lower value = longer integration)
        // Check Adafruit docs for mapping
        write8(Register.REG_ATIME.bVal, (byte)0xEB);
        // Set gain (0x00=1x, 0x01=4x, 0x02=16x, 0x03=60x)
        write8(Register.REG_CONTROL.bVal, (byte)0x01); // 4x gain
        // Wait for first integration cycle to complete
        // sleep(120);
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Color Sensor";
    }
    //Register map of the i2c device
    // Command bit
    private static final byte COMMAND_BIT = (byte)0x80;

    //Device Status enum that captures the current fault condition of the device
    public enum DeviceStatus{
        NOT_READY                (0),
        READY                    (1),
        CALIBRATING              (1 << 1),
        FAULT_BAD_READ           (1 << 5);
        private final int status;
        DeviceStatus(int status){
            this.status = status;
        }
    }

    public enum ReadData {
        ONLY_UPDATE_HEADING,
    }

    private void write8(byte reg, byte value) {
        deviceClient.write8((byte)(COMMAND_BIT | reg), value);
    }

    private byte read8(byte reg) {
        return deviceClient.read8((byte)(COMMAND_BIT | reg));
    }

    private int read16(byte regLow) {
        byte[] data = deviceClient.read((byte)(COMMAND_BIT | regLow), 2);
        int low = data[0] & 0xFF;
        int high = data[1] & 0xFF;
        return (high << 8) | low;
    }

    /** Writes an int to the i2c device
     @param reg the register to write the int to
     @param i the integer to write to the register
     */
    private void writeInt(final Register reg, int i){
        deviceClient.write(reg.bVal, TypeConversion.intToByteArray(i,ByteOrder.LITTLE_ENDIAN));
    }

    /**
     * Reads an int from a register of the i2c device
     * @param reg the register to read from
     * @return returns an int that contains the value stored in the read register
     */
    private int readInt(Register reg){
        return byteArrayToInt(deviceClient.read(reg.bVal,4), ByteOrder.LITTLE_ENDIAN);
    }

    /**
     * Converts a byte array to a float value
     * @param byteArray byte array to transform
     * @param byteOrder order of byte array to convert
     * @return the float value stored by the byte array
     */
    private float byteArrayToFloat(byte[] byteArray, ByteOrder byteOrder){
        return ByteBuffer.wrap(byteArray).order(byteOrder).getFloat();
    }

    /**
     * Reads a float from a register
     * @param reg the register to read
     * @return the float value stored in that register
     */
    private float readFloat(Register reg){
        return byteArrayToFloat(deviceClient.read(reg.bVal,4),ByteOrder.LITTLE_ENDIAN);
    }

    /**
     * Converts a float to a byte array
     * @param value the float array to convert
     * @return the byte array converted from the float
     */
    private byte [] floatToByteArray (float value, ByteOrder byteOrder) {
        return ByteBuffer.allocate(4).order(byteOrder).putFloat(value).array();
    }

    /**
     * Writes a byte array to a register on the i2c device
     * @param reg the register to write to
     * @param bytes the byte array to write
     */
    private void writeByteArray (Register reg, byte[] bytes){
        deviceClient.write(reg.bVal,bytes);
    }

    /**
     * Writes a float to a register on the i2c device
     * @param reg the register to write to
     * @param f the float to write
     */
    private void writeFloat (Register reg, float f){
        byte[] bytes = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putFloat(f).array();
        deviceClient.write(reg.bVal,bytes);
    }

    public void update() {
        // Read raw RGBC values
        rclear = read16(Register.REG_CDATAL.bVal);
        rred = read16(Register.REG_RDATAL.bVal);
        rgreen = read16(Register.REG_GDATAL.bVal);
        rblue = read16(Register.REG_BDATAL.bVal);

        // Simple normalization to get approximate RGB 0-255
        red = RGBmap(rred, redCalBlackValue, redCalWhiteValue);
        green = RGBmap(rgreen, greenCalBlackValue, greenCalWhiteValue)-0.5;
        blue = RGBmap(rblue, blueCalBlackValue, blueCalWhiteValue);
    }

    public double RGBmap(double x, int inlow, int inhigh){
        return ((x - inlow)/(inhigh-inlow)) * 255;
    }
}

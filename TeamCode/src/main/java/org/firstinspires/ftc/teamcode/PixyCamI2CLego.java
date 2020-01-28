package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;

/**
 *
 * Created by egarland 2020-01-28
 */
//@SuppressWarnings({"WeakerAccess", "unused"})

@I2cDeviceType
@DeviceProperties(name = "PixyCam Lego Mode", description = "PixyCam using the Lego I2c Protocol", xmlTag = "PIXYCAMLGO")
public class PixyCamI2CLego extends I2cDeviceSynchDevice<I2cDeviceSynch> implements HardwareDevice {

    private static final char[] HEX_ARRAY = "0123456789ABCDEF".toCharArray();

    public PixyCamI2CLego(I2cDeviceSynch i2cDeviceSynch) {
        super(i2cDeviceSynch, true);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();

    }


    byte[] read(int ireg, int creg) {
        return deviceClient.read(ireg, creg) ;
    }

    void write(int ireg, byte[] bytes) {
         deviceClient.write(ireg, bytes);
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "PixyCam";
    }


    public static String bytesToHex(byte[] bytes) {
        char[] hexChars = new char[bytes.length * 2];
        for (int j = 0; j < bytes.length; j++) {
            int v = bytes[j] & 0xFF;
            hexChars[j * 2] = HEX_ARRAY[v >>> 4];
            hexChars[j * 2 + 1] = HEX_ARRAY[v & 0x0F];
        }
        return new String(hexChars);
    }

}
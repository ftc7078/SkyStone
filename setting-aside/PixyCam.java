package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;

@I2cSensor(name = "MCP9808 Temperature Sensor", description = "Temperature Sensor from Adafruit", xmlTag = "MCP9808")
public class PixyCam extends I2cDeviceSynchDevice<I2cDeviceSynch>  {

    public PixyCam (I2cDeviceSynch deviceClient){
        super(deviceClient, true);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        return false;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Pixy";
    }

}
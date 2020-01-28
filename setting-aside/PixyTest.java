package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;


@Autonomous(name = "Pixy", group = "Tests")
public class PixyTest extends LinearOpMode {
    I2cDeviceSynch pixyReader;
    I2cAddr pixyAddress = I2cAddr.create7bit(0x54);
    byte[] readCache;

    @Override
    public void runOpMode() throws InterruptedException {
        pixyReader = hardwareMap.i2cDeviceSynch.get("pixy");

        pixyReader.engage();

        waitForStart();

        while(opModeIsActive()) {
            pixyReader.engage();
            byte[] sign1 = pixyReader.read(0x51, 5);
            byte[] sign2 = pixyReader.read(0x52,5);
            //notice the 0xff&sign1[x], the 0xff& does an absolute value on the byte
            //the sign1[x] gets byte 1 from the query, see above comments for more info
            telemetry.addData("X value of sign1", 0xff&sign1[1]);
            telemetry.addData("X value of sign2", 0xff&sign2[1]);
            telemetry.update();
        }
    }
}

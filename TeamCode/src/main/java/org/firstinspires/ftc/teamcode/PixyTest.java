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
    private static final char[] HEX_ARRAY = "0123456789ABCDEF".toCharArray();


    @Override
    public void runOpMode() throws InterruptedException {
        pixyReader = hardwareMap.i2cDeviceSynch.get("pixy");

        pixyReader.engage();
        int updates=0;

        waitForStart();

        while(opModeIsActive()) {
            pixyReader.engage();
	        byte[] sign1 = pixyReader.read(0x51, 5);
            byte[] sign2 = pixyReader.read(0x50, 5);
            //notice the 0xff&sign1[x], the 0xff& does an absolute value on the byte
            //the sign1[x] gets byte 1 from the query, see above comments for more info
            //telemetry.addData("A Center" , bytesToHex( sign1[0]));

            telemetry.addData("X Center" , toPercent(sign1[1]));

            telemetry.addData("Y Center" , toPercent( sign1[2]));
            telemetry.addData("Width" , toPercent( sign1[3]));
            telemetry.addData("Height" , toPercent( sign1[4]));
            updates++;
            telemetry.addData("Updates", updates);
            telemetry.update();

            sleep(100);
        }
    }

    public static String toPercent(byte aByte) {
        return String.format("%.1f", ((int)(aByte & 0xFF) / 2.55));
    }

    public static String bytesToHex(byte aByte) {
        return bytesToHex(new byte[]{aByte});
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

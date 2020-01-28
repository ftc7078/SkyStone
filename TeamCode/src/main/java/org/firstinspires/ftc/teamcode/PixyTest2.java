package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "Pixy2", group = "Tests")
public class PixyTest2 extends LinearOpMode {
    PixyCamI2CLego pixy;
    byte[] readCache;

    @Override
    public void runOpMode() throws InterruptedException {
        pixy = hardwareMap.get(PixyCamI2CLego.class, "pixy");


        waitForStart();

        while(opModeIsActive()) {
            for (int i = 0x51; i< 0x57; i++) {
                telemetry.addData("Register " + i, pixy.bytesToHex(pixy.read(i,8)));
            }

            byte[] sign1 = pixy.read(0x51, 5);
            byte[] sign2 = pixy.read(0x52,5);
            //notice the 0xff&sign1[x], the 0xff& does an absolute value on the byte
            //the sign1[x] gets byte 1 from the query, see above comments for more info
            telemetry.addData("X value of sign1", pixy.bytesToHex(sign1));
            telemetry.addData("X value of sign2", pixy.bytesToHex(sign2));
            telemetry.update();
        }
    }
}

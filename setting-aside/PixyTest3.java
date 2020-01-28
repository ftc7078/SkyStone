package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "Pixy3", group = "Tests")
public class PixyTest3 extends LinearOpMode {
    PixyCamI2CNative pixy;
    byte[] readCache;

    @Override
    public void runOpMode() throws InterruptedException {
        pixy = hardwareMap.get(PixyCamI2CNative.class, "pixy");


        waitForStart();

        while(opModeIsActive()) {
            pixy.write8(1, 0xc1ae );
            telemetry.addData("Byte 0", pixy.read8(0));
            telemetry.addData("Byte 1", pixy.read8(1));
            telemetry.addData("Byte 2", pixy.read8(2));
            telemetry.addData("Byte 3", pixy.read8(3));
            telemetry.addData("Byte 4", pixy.read8(4));
            telemetry.addData("Byte 5", pixy.read8(5));
            telemetry.addData("Byte 6", pixy.read8(6));
            telemetry.addData("Byte 7", pixy.read8(7));
            telemetry.addData("Byte 8", pixy.read8(8));
            telemetry.addData("Byte 9", pixy.read8(9));
            telemetry.addData("Byte 10", pixy.read8(10));
            telemetry.addData("Byte 11", pixy.read8(11));
            telemetry.addData("Byte 12", pixy.read8(12));
            telemetry.addData("Byte 13", pixy.read8(13));
            telemetry.update();
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Autonomous(name="Pixy Cam 2", group="Test")
public class PixyCamTest2 extends LinearOpMode {
    AnalogInput pixyCam;

    @Override
    public void runOpMode() throws InterruptedException {
        pixyCam = hardwareMap.analogInput.get("pixy");
        waitForStart();
        while (opModeIsActive()) {

            telemetry.addData("Pixy",pixyCam.getVoltage());
            telemetry.update();

        }

    }
}



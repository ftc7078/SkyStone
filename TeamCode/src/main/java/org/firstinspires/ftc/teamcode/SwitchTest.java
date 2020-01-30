package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;


@Autonomous(name = "Switch Test", group = "Tests")
public class SwitchTest extends LinearOpMode {
    DigitalChannel digitalTouch;  // Hardware Device Object

    @Override
    public void runOpMode() throws InterruptedException {
       int updates=0;
        digitalTouch = hardwareMap.get(DigitalChannel.class, "switch");

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        waitForStart();

        while(opModeIsActive()) {
        updates++;
            telemetry.addData("switch" , digitalTouch.getState());
            telemetry.addData("Updates", updates);
            telemetry.update();

            sleep(100);
        }
    }

}

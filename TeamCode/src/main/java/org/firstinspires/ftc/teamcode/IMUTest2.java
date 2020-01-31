package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="IMU2", group="Tests")

public class IMUTest2 extends LinearOpMode
{
    private MecanumDriveIMU mecanumDrive = new MecanumDriveIMU();
    private AtlasRobot robot = new AtlasRobot();

    @Override
    public void runOpMode() {

        mecanumDrive.init(hardwareMap, telemetry, this);
        robot.init(hardwareMap, telemetry, this);

        telemetry.addData("Status", "Not ready");
        telemetry.update();

        while (!mecanumDrive.readyToStart()) {
            telemetry.addData("Status", "Not ready");
            telemetry.update();
            sleep(50);
        }
        telemetry.addData("Status", "ready");
        telemetry.update();

        waitForStart();
        mecanumDrive.setTurnStart();
        while (opModeIsActive()) {
            telemetry.addData("Angle", mecanumDrive.getTurnedAngle()) ;
            telemetry.addData("Left", mecanumDrive.degreesLeft(-90)) ;

            telemetry.update();
            sleep(50);

        }

    }

}
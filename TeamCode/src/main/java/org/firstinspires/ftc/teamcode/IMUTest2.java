package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="IMU2To", group="Tests")

public class IMUTest2 extends LinearOpMode
{
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private AtlasRobot robot = new AtlasRobot();

    @Override
    public void runOpMode() {

        mecanumDrive.init(hardwareMap, telemetry, this);
        robot.init(hardwareMap, telemetry, this);



        while (!mecanumDrive.isReady() && opModeIsActive() ) {
            telemetry.addData("Status", "initializing");
            telemetry.update();
            sleep(50);
        }


        telemetry.addData("Status", "ready");
        telemetry.update();

        waitForStart();
        mecanumDrive.setTurnStart();
        while (opModeIsActive()) {
            Orientation currentOrientation = mecanumDrive.getOrientation();
            double diff = mecanumDrive.angleDifference(currentOrientation, mecanumDrive.orientationAtStart);



            telemetry.addData("Angle", mecanumDrive.getTurnedAngle()) ;
            telemetry.addData("Diff from start", diff) ;

            telemetry.update();



            mecanumDrive.turnTo(45, 0.5);
            sleep(2000);



        }

    }

}
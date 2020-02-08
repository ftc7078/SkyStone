package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Random;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="Random Turn2", group="Tests")

public class IMUTest2 extends LinearOpMode
{
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private AtlasRobot robot = new AtlasRobot();
    Random r = new java.util.Random();
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
        //double directions[] = {0,90,180,-90};
        double directions[] = {0,5,10,15};
        while (opModeIsActive()) {
            Orientation currentOrientation = mecanumDrive.getOrientation();
            double diff = mecanumDrive.angleDifference(currentOrientation, mecanumDrive.orientationAtStart);


            telemetry.addData("Angle", mecanumDrive.getTurnedAngle()) ;
            telemetry.addData("Diff from start", diff) ;

            telemetry.update();

            r.nextInt(4);
            double degrees = directions[r.nextInt(4)];
            mecanumDrive.turnTo(degrees, 0.1);
            sleep(500);
            System.out.println(String.format("After turnTo:  %.1f.  At: %.1f", degrees, mecanumDrive.getOrientation().firstAngle));
            System.out.println("turn2 slop error:  "+ (degrees - mecanumDrive.degreesFromStart()));
        }

    }

}
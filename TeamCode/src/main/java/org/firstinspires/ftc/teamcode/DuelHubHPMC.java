

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

import static java.lang.Math.sqrt;


@TeleOp(name="Teleop HPMC", group="Linear Opmode")

public class DuelHubHPMC extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private HPMC leftFrontMotor = null;
    private HPMC leftBackMotor = null;
    private HPMC rightFrontMotor = null;
    private HPMC rightBackMotor = null;
    private DcMotor rightManipulator = null;
    private DcMotor leftManipulator = null;
    private double[] motor = new double[4];
    private final int FL=0;
    private final int FR=1;
    private final int BL =2;
    private final int BR =3;
    private final double MSPEED=1.0;
    private final double SLOW=0.4;
    private final double MAX_SPEED=2800;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        leftFrontMotor = new HPMC( hardwareMap.get(DcMotor.class, "left_front"), MAX_SPEED) ;
        leftBackMotor = new HPMC( hardwareMap.get(DcMotor.class, "left_back"), MAX_SPEED);
        rightFrontMotor = new HPMC( hardwareMap.get(DcMotor.class, "right_front"), MAX_SPEED);
        rightBackMotor = new HPMC( hardwareMap.get(DcMotor.class, "right_back"), MAX_SPEED);




        leftManipulator = hardwareMap.get(DcMotor.class, "left_manipulator");
        rightManipulator = hardwareMap.get(DcMotor.class, "right_manipulator");



        // Set Power Levels to zero
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftManipulator.setPower(0);
        rightManipulator.setPower(0);



        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);



        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry

            // Mecanum Mode uses left stick to go forwardSpeed and turn.

            double speed = 1;
             if ( gamepad1.left_trigger > 0.5 ) {
                 speed = 0.6;
             }
             if ( gamepad1.right_trigger > 0.5) {
                 speed = 0.3;
             }

            setMotors(gamepad1.left_stick_x,gamepad1.left_stick_y, gamepad1.right_stick_x, speed);


            boolean pull = gamepad2.x;
            boolean push = gamepad2.b;
            if (pull) {
                leftManipulator.setPower(MSPEED);
                rightManipulator.setPower(-MSPEED);
                telemetry.addData("Manipulator Motors", "Pulling");
            } else if (push) {
                leftManipulator.setPower(-MSPEED);
                rightManipulator.setPower(MSPEED);
                telemetry.addData("Manipulator Motors", "Pushing");
            } else {
                telemetry.addData("Manipulator Motors", "Idle");
                leftManipulator.setPower(0);
                rightManipulator.setPower(0);
            }

            leftBackMotor.tickSleep();
            telemetry.addData("Left/Right Stick", "LX (%.2f), LY (%.2f), RX (%.2f), RY (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);

            telemetry.update();
        }
    }



    void setMotors(double x, double y, double rot, double speed) {
        double theta = Math.atan2(x,y) - Math.PI/4;  //finds the angle of the joystick and turns it by pi/4 radians or 45 degrees
        rot = 1*rot;  //scales rotation factor
        double magnitude = Math.hypot(x,y);  //finds the magnitude of the joystick input by the Pythagorean theorem
        telemetry.addData("Mag", "Mag (%.2f) Rot(%.2f)",magnitude, rot);
        if (magnitude > 1) {magnitude = 1;}

        double div =  magnitude + Math.abs(rot);
        if (div > 1) {
            magnitude = magnitude / div;
            rot = rot / div; // subtracts rot from the magnitude to make room for it and scales the magnitude
        }
        double u = Math.cos(theta)*magnitude; //finds the input to one set of wheels
        double v = Math.sin(theta)*magnitude; //finds the input to the other set of wheels

        telemetry.addData("Calculated XY", "nX (%.2f), nY (%.2f)", u, v);

        double u2 = u * u;
        double v2 = v * v;
        double twosqrt2 = 2.0 * sqrt(2.0);
        double subtermx = 2.0 + u2 - v2;
        double subtermy = 2.0 - u2 + v2;
        double termx1 = subtermx + u * twosqrt2;
        double termx2 = subtermx - u * twosqrt2;
        double termy1 = subtermy + v * twosqrt2;
        double termy2 = subtermy - v * twosqrt2;
        if (termx1 < 0) {termx1=0;}
        if (termx2 < 0) {termx2=0;}
        if (termy1 < 0) {termy1=0;}
        if (termy2 < 0) {termy2=0;}
        double newX = 0.5 * sqrt(termx1) - 0.5 * sqrt(termx2);
        double newY = 0.5 * sqrt(termy1) - 0.5 * sqrt(termy2);

        //from here on is just setting motor values



        motor[FR] =  newX + rot;
        motor[BL] =  newX - rot;
        motor[FL] =  newY + rot;
        motor[BR] =  newY - rot;
        if (speed < 1) {
            motor[FR] = motor[FR]*speed;
            motor[FL] = motor[FL]*speed;
            motor[BL] = motor[BL]*speed;
            motor[BR] = motor[BR]*speed;
        }
        leftFrontMotor.setPower(motor[FL]);
        leftBackMotor.setPower(motor[BL]);
        rightFrontMotor.setPower(motor[FR]);
        rightBackMotor.setPower(motor[BR]);

        telemetry.addData("Motors", "lf (%.2f), rf (%.2f), lb (%.2f), rb (%.2f)", motor[FL], motor[FR], motor[BL], motor[BR]);
    }



}

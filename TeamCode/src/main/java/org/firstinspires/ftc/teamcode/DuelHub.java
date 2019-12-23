

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import static java.lang.Math.sqrt;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;


@TeleOp(name="DuelHub", group="Linear Opmode")

public class DuelHub extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;
    private DcMotor rightManipulator = null;
    private DcMotor leftManipulator = null;
    private DistanceSensor sensorRange;
    private double[] motor = new double[4];

    private final int FL=0;
    private final int FR=1;
    private final int BL =2;
    private final int BR =3;
    private final double SLOW=0.6;

    private final double MSPEED=1.0;
    private double updates = 0;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        //test live UnsupportedOperationException
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back");



        leftManipulator = hardwareMap.get(DcMotor.class, "left_manipulator");
        rightManipulator = hardwareMap.get(DcMotor.class, "right_manipulator");
        sensorRange = hardwareMap.get(DistanceSensor.class, "lazorz");



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

            // Mecanum Mode uses left stick to go forward and turn.
            boolean boost = gamepad1.right_trigger>0.5;

            setMotors(gamepad1.left_stick_x,gamepad1.left_stick_y, gamepad1.right_stick_x, boost);

            boolean pull = gamepad1.x;
            boolean push = gamepad1.b;
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



            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "lf (%.2f), rf (%.2f), lb (%.2f), rb (%.2f)", v1, v2, v3, v4);
            telemetry.addData("Left/Right Stick", "LX (%.2f), LY (%.2f), RX (%.2f), RY (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("range", String.format(Locale.US,
                    "%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
            //telemetry.update();
            sleep(1);

            telemetry.update();


            telemetry.addData("Updates Per Second", "%.1f", updates/runtime.seconds() );

        }
    }


    void setMotors(double x, double y, double rot, boolean boost) {
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
            if (!boost) {
                motor[FR] = motor[FR]*SLOW;
                motor[FL] = motor[FL]*SLOW;
                motor[BL] = motor[BL]*SLOW;
                motor[BR] = motor[BR]*SLOW;
            }
            leftFrontMotor.setPower(motor[FL]);
            leftBackMotor.setPower(motor[BL]);
            rightFrontMotor.setPower(motor[FR]);
            rightBackMotor.setPower(motor[BR]);
            telemetry.addData("Motors", "lf (%.2f), rf (%.2f), lb (%.2f), rb (%.2f)", motor[FL], motor[FR], motor[BL], motor[BR]);


    }


}

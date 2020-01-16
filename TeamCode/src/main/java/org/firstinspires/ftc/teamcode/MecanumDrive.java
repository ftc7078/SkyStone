/*7078 mecanumm drive code */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.sqrt;

public class MecanumDrive {
    private HPMC[] motors = new HPMC[4];
    private double[] power = new double[4];
    private final double MOTOR_SPEED = 2800;
    private final long COUNT_PER_INCH = 60;
    private final long COUNT_PER_INCH_STRAFING = 60;
    private final long COUNT_PER_DEGREE = 13;


    private final int FL = 0;
    private final int FR = 1;
    private final int BL = 2;
    private final int BR = 3;
    private Telemetry telemetry;
    private LinearOpMode opMode;



    enum MoveDirection {FORWARD, BACKWARD, LEFT, RIGHT}


    //For Smart Ticks
    private long nextWake = 0;
    static long tickTime = 50; //in milliseconds


    public void init(HardwareMap hardwareMap, Telemetry telemetryIn, LinearOpMode opModeIn) {

        telemetry = telemetryIn;
        opMode = opModeIn;
        motors[FL] = new HPMC(hardwareMap, "left_front", MOTOR_SPEED);
        motors[BL] = new HPMC(hardwareMap, "left_back", MOTOR_SPEED);
        motors[FR] = new HPMC(hardwareMap, "right_front", MOTOR_SPEED);
        motors[BR] = new HPMC(hardwareMap, "right_back", MOTOR_SPEED);




        // Set Power Levels to zero
        for (HPMC motor : motors) {
            motor.setPower(0);
            motor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

        motors[FL].setLabel("FL");
        motors[BL].setLabel("BL");
        motors[FR].setLabel("FR");
        motors[BR].setLabel("BR");


        // Reverse the motors that runs backwards when connected directly to the battery
        motors[FL].setDirection(DcMotor.Direction.FORWARD);
        motors[BL].setDirection(DcMotor.Direction.FORWARD);
        motors[FR].setDirection(DcMotor.Direction.REVERSE);
        motors[BR].setDirection(DcMotor.Direction.REVERSE);

    }


    public void tickSetup() {
        long now = System.nanoTime();
        nextWake = now + tickTime * 1000000;
    }

    public void tickSleep() {
        long now = System.nanoTime();
        nextWake = nextWake + tickTime * 1000000;
        if (nextWake < now) {
            nextWake = now + tickTime * 1000000;
            double msLate = (now - nextWake) / 1000000;
            if (msLate > 100) {
                System.out.println(String.format("Either this is the first tick or something is wrong: %.1f ms late", msLate));
                throw new RuntimeException(String.format("Can't keep up: tick took %.1f ms too long", msLate));
            }
            return;
        }
        long sleepTime = (int) Math.floor((nextWake - now) / 1000000);
        if (false) {//for debugging
            System.out.println("Sleeping: " + sleepTime);
        }

        try {
            Thread.sleep(sleepTime);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    int getCurrentPosition() {
        int position = 0;
        for (int i = 0; i < 4; i++) {
            position += motors[i].getCurrentPosition();
        }
        return ((int) (position / 4));
    }

    void setMotors(double x, double y, double rot, double slowdownFactor) {
        double theta = Math.atan2(x, y) - Math.PI / 4;  //finds the angle of the joystick and turns it by pi/4 radians or 45 degrees
        rot = 1 * rot;  //scales rotation factor
        double magnitude = Math.hypot(x, y);  //finds the magnitude of the joystick input by the Pythagorean theorem
        telemetry.addData("Mag", "Mag (%.2f) Rot(%.2f)", magnitude, rot);


        if (magnitude > 1) {
            magnitude = 1;
        }

        double div = magnitude + Math.abs(rot);
        if (div > 1) {
            magnitude = magnitude / div;
            rot = rot / div; // subtracts rot from the magnitude to make room for it and scales the magnitude
        }
        double u = Math.cos(theta) * magnitude; //finds the input to one set of wheels
        double v = Math.sin(theta) * magnitude; //finds the input to the other set of wheels

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
        if (termx1 < 0) {
            termx1 = 0;
        }
        if (termx2 < 0) {
            termx2 = 0;
        }
        if (termy1 < 0) {
            termy1 = 0;
        }
        if (termy2 < 0) {
            termy2 = 0;
        }
        double newX = 0.5 * sqrt(termx1) - 0.5 * sqrt(termx2);
        double newY = 0.5 * sqrt(termy1) - 0.5 * sqrt(termy2);

        //from here on is just setting motors values
        if (false) {
            newX = newX * slowdownFactor;
            newY = newY * slowdownFactor;
        }
        power[FR] = -(newX + rot);
        power[BL] = -(newX - rot);
        power[FL] = newY + rot;
        power[BR] = newY - rot;
        for (int i = 0; i < 4; i++) {
            power[i] = power[i]*slowdownFactor;
            motors[i].setPower(power[i]);
        }

        telemetry.addData("Motors", "lf (%.2f), rf (%.2f), lb (%.2f), rb (%.2f)", power[FL], power[FR], power[BL], power[BR]);
    }

    void forward(double distance, double power) {
        move(distance, power, MoveDirection.FORWARD);
    }
    void backward(double distance, double power) {
        move(distance, power, MoveDirection.BACKWARD);
    }
    void leftStrafe (double distance, double power) {
        move(distance, power, MoveDirection.LEFT);
    }
    void rightStrafe (double distance, double power) {
        move(distance, power, MoveDirection.RIGHT);
    }
    void leftTurn(double degrees, double power) {
        turn(degrees, power, MoveDirection.LEFT);
    }
    void rightTurn(double degrees, double power) {
        turn(degrees, power, MoveDirection.RIGHT);
    }




    void turn(double degrees, double power, MoveDirection direction) {
        long distance = (long) degrees * COUNT_PER_DEGREE;
        HPMC.Direction FL_Direction = HPMC.Direction.FORWARD;
        HPMC.Direction FR_Direction = HPMC.Direction.FORWARD;
        switch (direction) {

            case LEFT:
                FL_Direction = HPMC.Direction.REVERSE;
            case RIGHT:
                FR_Direction = HPMC.Direction.REVERSE;
        }
        for (int i : new int[]{FL, BL}) {
            motors[i].smoothMoveSetup(distance, power, power * 5, FL_Direction, true);
        }
        for (int i : new int[]{FR, BR}) {
            motors[i].smoothMoveSetup(distance, power, power * 5, FL_Direction, true);
        }
        smTickUntilDone();
    }

    void move(double inches, double power, MoveDirection direction) {
        long distance = 0;
        if (inches < 0) {
            inches = Math.abs(inches);
        }
        switch (direction) {
            case FORWARD:
            case BACKWARD:
                distance = (long) inches * COUNT_PER_INCH;
                HPMC.Direction motorDirection = HPMC.Direction.FORWARD;
                if (direction == MoveDirection.BACKWARD) {
                    motorDirection = HPMC.Direction.REVERSE;
                }
                for (HPMC motor : motors) {
                    motor.smoothMoveSetup(distance, power, power * 10, motorDirection, true);
                }
                break;
            case LEFT:
            case RIGHT:
                distance = (long) inches * COUNT_PER_INCH_STRAFING;
                HPMC.Direction FL_Direction = HPMC.Direction.FORWARD;
                HPMC.Direction FR_Direction = HPMC.Direction.REVERSE;
                if (direction == MoveDirection.LEFT) {
                    FL_Direction = HPMC.Direction.REVERSE;
                } else {
                    FR_Direction = HPMC.Direction.REVERSE;
                }
                for (int i : new int[]{FL, BR}) {
                    motors[i].smoothMoveSetup(distance, power, power * 5, FL_Direction, true);
                }
                for (int i : new int[]{FR, BL}) {
                    motors[i].smoothMoveSetup(distance, power, power * 5, FL_Direction, true);
                }
                break;
        }
        smTickUntilDone();
    }

    void smTickUntilDone() {
        boolean done = false;
        tickSetup();
        while (!done && opMode.opModeIsActive()) {
            done = true;
            for (HPMC motor : motors) {
                //smTick returns true if we need to keep going, false if we're done moving
                //Keep going if any motor is not done
                if (motor.smTick()) {
                    done = true;
                }
            }
            tickSleep();
        }
    }

    void forward_old(double distance, double speed, int accelerationTicks) {
        HPMC.Direction direction = HPMC.Direction.FORWARD;
        if (distance < 0) {
            distance = Math.abs(distance);
            direction = HPMC.Direction.REVERSE;
        }
        for (HPMC motor : motors) {
            motor.smoothMoveSetup(distance, speed, accelerationTicks, direction, true);
        }
        boolean notDone = true;
        while (notDone && opMode.opModeIsActive()) {
            notDone = false;
            for (HPMC motor : motors) {
                //Returns true if we need to keep going, false if we're done moving
                //Keep going if any motor is not done
                if (motor.smTick()) {
                    notDone = true;
                }
            }
            tickSleep();
        }

    }
}

/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.sqrt;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Self Test", group ="Concept")

public class SelfTest extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private HPMC[] motors = new HPMC[4];
    private HPMC rightManipulator = null;
    private HPMC leftManipulator = null;
    private DistanceSensor sensorRange;
    private double[] power = new double[4];
    private final double MOTOR_SPEED = 2800;

    private final int FL=0;
    private final int FR=1;
    private final int BL =2;
    private final int BR =3;
    private final double SLOW=0.6;

    private final double MSPEED=1.0;
    private double updates = 0;

    @Override public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        //test live UnsupportedOperationException
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        motors[FL] = new HPMC(hardwareMap,"left_front", MOTOR_SPEED);
        motors[BL] =  new HPMC (hardwareMap, "left_back", MOTOR_SPEED);
        motors[FR] = new HPMC (hardwareMap, "right_front", MOTOR_SPEED);
        motors[BR] = new HPMC(hardwareMap, "right_back", MOTOR_SPEED);

        // Set Power Levels to zero
        motors[FL].setPower(0);
        motors[BL].setPower(0);
        motors[FR].setPower(0);
        motors[BR].setPower(0);

        motors[FL].setLabel("FL");
        motors[BL].setLabel("BL");
        motors[FR].setLabel("FR");
        motors[BR].setLabel("BR");



        // Reverse the motors that runs backwards when connected directly to the battery
        motors[FL].setDirection(DcMotor.Direction.FORWARD);
        motors[BL].setDirection(DcMotor.Direction.FORWARD);
        motors[FR].setDirection(DcMotor.Direction.REVERSE);
        motors[BR].setDirection(DcMotor.Direction.REVERSE);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        waitForStart();
        System.out.println("Starting forward 10");
        motors[FL].motor.setPower(.5);
        sleep(3000);
        motors[FL].motor.setPower(0);
        motors[BL].motor.setPower(0.5);
        sleep(3000);
        motors[BL].motor.setPower(0);
        motors[BR].motor.setPower(0.5);

        sleep(3000);
        motors[BR].motor.setPower(0);
        motors[FR].motor.setPower(0.5);

        sleep(3000);
        motors[FR].motor.setPower(0);


        //System.out.println("Starting backward 10");
        //forward( -10, 0.4, 10);
        System.out.println("Done");
        telemetry.addData("Status", "Done");
        telemetry.update();

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

        //from here on is just setting motors values



        power[FR] =  newX + rot;
        power[BL] =  newX - rot;
        power[FL] =  newY + rot;
        power[BR] =  newY - rot;
        if (!boost) {
            for (int i=0;i<4;i++) {
                power[i] = power[i]*SLOW;
            }
        }
        for (int i = 0; i<4; i++) {
            motors[i].setPower(power[i]);
        }
        telemetry.addData("Motors", "lf (%.2f), rf (%.2f), lb (%.2f), rb (%.2f)", power[FL], power[FR], power[BL], power[BR]);


    }

    void forwardSpeed(double speed) {
        setMotors( 0 , -speed, 0, true);
    }
    void forward(double distance, double speed, int accelerationTicks){
            HPMC.Direction direction = HPMC.Direction.FORWARD;
            if (distance < 0) {
                distance = Math.abs(distance);
                direction = HPMC.Direction.REVERSE;
            }
            for (HPMC motor : motors) {
                motor.smoothMoveSetup(distance, speed, accelerationTicks, direction, true);
            }
            boolean notDone = true;
            while (notDone) {
                for (int i = 0; i<4; i++) {
                    HPMC motor = motors[i];
                    notDone = false;
                    if (motor.smTick()) {
                        notDone = true;
                    }
                }
                motors[FR].tickSleep();

            }
            /*
        for(int i=0; i<20; i++) {
            forwardSpeed(i/20);
            smartSleep();
        }
        for(int i=0; i<20; i++) {
            forwardSpeed(i/20);
            smartSleep();
        }
        for(int i=0; i<20; i++) {
            forwardSpeed(1-(i/20));
            smartSleep();
        }*/
    }
}

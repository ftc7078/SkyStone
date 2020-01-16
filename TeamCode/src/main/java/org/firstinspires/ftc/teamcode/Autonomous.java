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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.sqrt;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group ="Concept")

public class Autonomous extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightManipulator = null;
    private DcMotor leftManipulator = null;
    private MecanumDrive mecanumDrive = new MecanumDrive();

    private Servo   foundationLeft;
    private Servo   foundationRight;


    @Override public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        //test live UnsupportedOperationException
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        mecanumDrive.init(hardwareMap, telemetry, this);



        foundationLeft = hardwareMap.get(Servo.class,"foundationLeft");
        foundationRight = hardwareMap.get(Servo.class,"foundationRight");


        leftManipulator = hardwareMap.get(DcMotor.class, "left_manipulator");
        rightManipulator = hardwareMap.get(DcMotor.class, "right_manipulator");



        // Set Power Levels to zero

        leftManipulator.setPower(0);
        rightManipulator.setPower(0);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        waitForStart();

        sleep(100000);
        foward( 1, 0.4);
    }
    void blueLoad() {
        //Scan blocks to decide wich block(should be 3 cases)
        mecanumDrive.forward(36,.7);//to pick up the blocks
        //run colector until sensor is triggered
        mecanumDrive.backward(6, .7);
        mecanumDrive.leftTurn(90, 1);
        mecanumDrive.backward(72, .7);
        //drop arm
        foundationMover(false);

        //drop off block
        mecanumDrive.forward(24,.7);
        mecanumDrive.leftTurn(30, .7);
        //lift arm
        foundationMover(true);

        mecanumDrive.leftStrafe(18, 1);
        mecanumDrive.leftTurn(60, 1);
        mecanumDrive.forward(24,.7);
    }

    void blueBuild() {
        mecanumDrive.backward(30,1);
        foundationMover(false);
        mecanumDrive.forward(24,.7);
        mecanumDrive.leftTurn(30, 1);
        foundationMover(true);
        mecanumDrive.leftStrafe(18,1);
        mecanumDrive.leftTurn(30, .5);
        mecanumDrive.forward(24,.7);
    }

    void foundationMover(boolean up ){
        if (up){
            foundationLeft.setPosition(1);
            foundationRight.setPosition(0);
        } else {
            foundationLeft.setPosition(0);
            foundationRight.setPosition(1);
        }
    }

    void foward(double distance, double speed){
        int count_to_travel = (int) (distance * 10);
        int start_position = mecanumDrive.getCurrentPosition();
        int end_position = start_position + count_to_travel;
        while ( mecanumDrive.getCurrentPosition() < end_position) {
            mecanumDrive.setMotors(1,0,0,speed);
            mecanumDrive.tickSleep();
        }
    }
}

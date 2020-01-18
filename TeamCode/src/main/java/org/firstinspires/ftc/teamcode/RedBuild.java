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


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RedBuild", group ="Concept")

public class RedBuild extends LinearOpMode {

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

        mecanumDrive.forward(12,.5);
        mecanumDrive.rightTurn(45, .5);
        mecanumDrive.forward(20, .5);
        mecanumDrive.leftTurn(45, .5);
        mecanumDrive.forward(6, .5);
        foundationMover(false);
        sleep(2000);
        mecanumDrive.backward(36,.5);
        mecanumDrive.rightTurn(270, .5);
        mecanumDrive.forward(24, .5);
        //mecanumDrive.rightStrafe(12, .5);
        foundationMover(true);
        mecanumDrive.backward(6, .5);
        mecanumDrive.leftTurn(180, .5);
        mecanumDrive.leftStrafe(24, .5);
        mecanumDrive.forward(42,.5);
    }

    void foundationMover(boolean up ){
        if (up){
            foundationLeft.setPosition(.35);
            foundationRight.setPosition(.65);
        } else {
            foundationLeft.setPosition(.55);
            foundationRight.setPosition(.45);
        }
    }

}

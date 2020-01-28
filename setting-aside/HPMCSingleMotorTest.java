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
import com.qualcomm.robotcore.util.ElapsedTime;



@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="HPMC Single Motor Test", group ="Tests")

public class HPMCSingleMotorTest extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private HPMC motor = null;
    private double updates = 0;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        //test live UnsupportedOperationException
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motor = new HPMC(hardwareMap, "right_back", 2800);

        // Set Power Levels to zero
        motor.setPower(0);
        motor.setLabel("Motor");


        // Reverse the motor that runs backwards when connected directly to the battery
        motor.setDirection(DcMotor.Direction.FORWARD);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        runtime.reset();


        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();


        move(100, 0.5, 50, HPMC.Direction.FORWARD, true);
        //move(100, 0.9, 20, HPMC.Direction.REVERSE, true);

        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }


    void move(double distance, double power, long accelerationTicks,  HPMC.Direction direction, Boolean endStopped) {
        System.out.println("Starting moving "+direction.toString());
        motor.smoothMoveSetup(distance, power, accelerationTicks, accelerationTicks, direction, endStopped);

        int tick = 0;
        while (opModeIsActive() && motor.smTick() ) {
            tick++;
            motor.tickSleep();
            telemetry.addData("Tick" , tick);
            telemetry.addData("SMStatus", motor.getSMStatus() );
            System.out.println(String.format("SMStatus: %s", motor.getSMStatus()));
            telemetry.addData( "Update Speed: ", motor.getUpdatesPerSecond());
            telemetry.update();
        }
    }
}



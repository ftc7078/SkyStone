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


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Arc Test", group ="Tests")

public class ArcTest extends LinearOpMode {

    private MecanumDrive mecanumDrive = new MecanumDrive();
    private AtlasRobot robot = new AtlasRobot();

    @Override
    public void runOpMode() {

        mecanumDrive.init(hardwareMap, telemetry, this);
        robot.init(hardwareMap, telemetry, this);

        telemetry.addData("Status", "Initialized");

        waitForStart();

        //mecanumDrive.backward(4, 0.5);
        mecanumDrive.arcMove( 2, -90, .8, MecanumDrive.MoveDirection.RIGHT, false, true);
        //mecanumDrive.arcMove( 2, 180, .8, MecanumDrive.MoveDirection.LEFT, false, true);
        //mecanumDrive.arcMove( 2, 180, .8, MecanumDrive.MoveDirection.RIGHT, true, true);
       // mecanumDrive.arcMove( 2, 180, .8, MecanumDrive.MoveDirection.LEFT, true, true);



        //mecanumDrive.arcMove( 1, 90, .5, MecanumDrive.MoveDirection.LEFT, false, true);

        pause();



        status( "Done");

    }

    void pause() {
        sleep(100);
    }
    void status(String string) {
        telemetry.addData("Status", string);
        telemetry.update();
    }
}


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


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Red Skystone", group ="Concept")

public class RedSkystone extends LinearOpMode implements MecanumDriveIMU.TickCallback {

    private MecanumDriveIMU mecanumDrive = new MecanumDriveIMU();
    private AtlasRobot robot = new AtlasRobot();
    private TensorflowDetector tf;
    enum RedScan {LEFT, MIDDLE, RIGHT}
    int updates;

    @Override
    public void runOpMode() {
        tf = new TensorflowDetector();
        tf.init(hardwareMap, telemetry, this);

        while (!isStarted() ) {
            telemetry.addData("path", tf.choosePath());
            telemetry.update();
            sleep(100);
        }

        mecanumDrive.init(hardwareMap, telemetry, this);
        robot.init(hardwareMap, telemetry, this);
        telemetry.addData("Status", "Initialized");
        mecanumDrive.setupTickCallback(this);

        waitForStart();
        robot.inRamp.setPosition(1);
        int path = 0;
        int loops = 0;
        for (int i = 1; i<60; i++) {
            path = tf.choosePath();
            loops=i;
            if (path > 0) {
                break;
            }
            telemetry.addData("loop", i);
            telemetry.update();
            sleep(50);
        }
        if (path==0) {
            path = 1;
        }

        if (path == 1) {
            redScan(RedScan.RIGHT);
        }else if (path == 2) {
            redScan(RedScan.MIDDLE);
        }else if (path == 3) {
            redScan(RedScan.LEFT);
        }else {}
    }

    void redScan(RedScan output) {
        switch (output) {
            case RIGHT:
                mecanumDrive.backward(22, 1);
                robot.setManipulator(AtlasRobot.ManipulatorDirection.IN, true);
                mecanumDrive.arcMove( 8, -90, 1, MecanumDriveIMU.MoveDirection.LEFT, false, true);

                break;
            case MIDDLE:

                break;
            case LEFT:

                break;

        }

    }

    public void tickCallback() {
        updates++;
        if ( robot.switchPressed() ) {
            robot.manipulatorAutostop();
        }
        telemetry.addData("I'm running callback code rightnow!", updates);
        telemetry.update();
    }
}
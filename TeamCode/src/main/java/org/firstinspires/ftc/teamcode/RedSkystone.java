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

public class RedSkystone extends LinearOpMode implements MecanumDrive.TickCallback {

    private MecanumDrive mecanumDrive = new MecanumDrive();
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
                mecanumDrive.backward(22, .8);
                //Pick up first skystone
                robot.setManipulator(AtlasRobot.ManipulatorDirection.IN, true);
                mecanumDrive.arcMove( 8, -90, .5, MecanumDrive.MoveDirection.LEFT, false, true);
                mecanumDrive.leftStrafe(14,1);
                sleep(400);
                mecanumDrive.turnTo(-90,.05);
                //crossing under bridger for the first time
                mecanumDrive.backward(84,.8);
                mecanumDrive.turnTo(180,.3);
                mecanumDrive.forward(7,.3);
                //move foundation
                moveFoundation();
                mecanumDrive.backward(75,.8);
                mecanumDrive.turnTo(45,.3);
                robot.setManipulator(AtlasRobot.ManipulatorDirection.IN,true);
                mecanumDrive.backward(17,.3);
                mecanumDrive.forward(17,.3);
                mecanumDrive.turnTo(90,.3);
                mecanumDrive.forward(100,.9);
                robot.setManipulator(AtlasRobot.ManipulatorDirection.IN);
                mecanumDrive.turnTo(90,.3);
                //park
                mecanumDrive.backward(72-27,.8);

                break;
            case MIDDLE:
                mecanumDrive.diagonal(8,-.5,MecanumDrive.MoveDirection.RIGHT,true);
                mecanumDrive.backward(14, 1);
                robot.setManipulator(AtlasRobot.ManipulatorDirection.IN, true);
                mecanumDrive.arcMove( 8, -90, 1, MecanumDrive.MoveDirection.RIGHT, false, true);
                mecanumDrive.leftStrafe(12,1);
                mecanumDrive.backward(92,1);
                mecanumDrive.turnTo(180,1);
                mecanumDrive.forward(6,1);
                robot.foundationMover(false);
                sleep(2000);
                mecanumDrive.tankTurnStart(MecanumDrive.MoveDirection.RIGHT, -1);
                robot.setManipulator(AtlasRobot.ManipulatorDirection.IN);
                while(opModeIsActive() && mecanumDrive.degreesFromStart() < 90 ) {
                    sleep(50);
                }
                robot.setManipulator(AtlasRobot.ManipulatorDirection.STOP);
                robot.foundationMover(true);
                //mecanumDrive.backward(6,1);
                mecanumDrive.leftStrafe(8, 5);
                sleep(100);
                mecanumDrive.turnTo(90,1);

                break;
            case LEFT:

                break;
        }

    }
    void moveFoundation() {
        robot.foundationMover(false);
        sleep(1000);
        robot.setManipulator(AtlasRobot.ManipulatorDirection.IN);
        mecanumDrive.freeWheel(-0.1, -1, -0.1, -1, 24);
        mecanumDrive.freeWheel(0.2, -1, 0.2, -1, 15);

        robot.setManipulator(AtlasRobot.ManipulatorDirection.STOP);
        robot.foundationMover(true);
        //mecanumDrive.backward(6,1);
        mecanumDrive.leftStrafe(14, .8);
        sleep(200);
        mecanumDrive.turnTo(90, .05);
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
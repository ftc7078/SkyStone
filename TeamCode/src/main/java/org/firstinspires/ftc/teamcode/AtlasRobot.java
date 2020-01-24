/*7078 mecanumm drive code */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.sqrt;

public class AtlasRobot {

    private Telemetry telemetry;
    private LinearOpMode opMode;

    enum MoveDirection {FORWARD, BACKWARD, LEFT, RIGHT}

    private DcMotor rightManipulator = null;
    private DcMotor leftManipulator = null;
    private Servo foundationLeft;
    private Servo foundationRight;
    Servo capstone;
    enum CapstonePosition { UP, MIDDLE, DOWN};
    enum ManipulatorDireciton { IN, OUT, STOP};


    public void init(HardwareMap hardwareMap, Telemetry telemetryIn, LinearOpMode opModeIn) {

        telemetry = telemetryIn;
        opMode = opModeIn;

        foundationLeft = hardwareMap.get(Servo.class, "foundationLeft");
        foundationRight = hardwareMap.get(Servo.class, "foundationRight");
        capstone = hardwareMap.get(Servo.class, "capstone");


        leftManipulator = hardwareMap.get(DcMotor.class, "left_manipulator");
        rightManipulator = hardwareMap.get(DcMotor.class, "right_manipulator");

        leftManipulator.setPower(0);
        rightManipulator.setPower(0);

        // Reverse the motors that runs backwards when connected directly to the battery

    }

    void setCapstone(CapstonePosition position) {
        switch (position) {
            case UP:
                capstone.setPosition(0.1);
                break;
            case MIDDLE:
                capstone.setPosition(0.5);
                break;
            case DOWN:
                capstone.setPosition(1);
                break;
        }
    }

    void foundationMover(boolean up ){
        if (up){
            foundationLeft.setPosition(.35);
            foundationRight.setPosition(.65);
        } else {
            foundationLeft.setPosition(.6);
            foundationRight.setPosition(.4);
        }
    }

    void setManipulator(ManipulatorDireciton direction) {
        switch (direction) {
            case IN:
                leftManipulator.setPower(1);
                rightManipulator.setPower(-1);
                break;
            case OUT:
                leftManipulator.setPower(-1);
                rightManipulator.setPower(1);
                break;
            case STOP:
                leftManipulator.setPower(0);
                rightManipulator.setPower(0);
                break;

        }
    }
}
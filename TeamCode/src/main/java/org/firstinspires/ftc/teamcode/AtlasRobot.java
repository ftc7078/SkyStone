/*7078 mecanumm drive code */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AtlasRobot {

    private Telemetry telemetry;
    private LinearOpMode opMode;

    enum MoveDirection {FORWARD, BACKWARD, LEFT, RIGHT}

    private DcMotor rightManipulator = null;
    private DcMotor leftManipulator = null;
    private Servo foundationLeft;
    private Servo foundationRight;
    Servo capstone;
    Servo inRamp;
    enum CapstonePosition { UP, MIDDLE, DOWN}
    enum ManipulatorDirection { IN, OUT, STOP}

    public void init(HardwareMap hardwareMap, Telemetry telemetryIn, LinearOpMode opModeIn) {

        telemetry = telemetryIn;
        opMode = opModeIn;

        foundationLeft = hardwareMap.get(Servo.class, "foundationLeft");
        foundationRight = hardwareMap.get(Servo.class, "foundationRight");
        capstone = hardwareMap.get(Servo.class, "capstone");
        inRamp = hardwareMap.get(Servo.class, "in_ramp");

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
                capstone.setPosition(0.62);
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

    void setManipulator(ManipulatorDirection direction) {
        switch (direction) {
            case IN:
                inRamp.setPosition(1);
                leftManipulator.setPower(-1);
                rightManipulator.setPower(1);
                break;
            case OUT:
                inRamp.setPosition(0);
                leftManipulator.setPower(1);
                rightManipulator.setPower(-1);
                break;
            case STOP:
                leftManipulator.setPower(0);
                rightManipulator.setPower(0);
                inRamp.setPosition(0);
                break;
        }
    }
}

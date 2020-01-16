

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="ManipulatorTest", group="Linear Opmode")

public class ManipulatorTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightManipulator = null;
    private DcMotor leftManipulator = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
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

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry

            // Mecanum Mode uses left stick to go forwardSpeed and turn.
            boolean pull = gamepad1.x;
            boolean push = gamepad1.b;

            // Send calculated power to wheels

            if (pull) {
                leftManipulator.setPower(1);
                rightManipulator.setPower(-1);
                telemetry.addData("Motors", "Pulling");

            } else if (push) {
                leftManipulator.setPower(-1);
                rightManipulator.setPower(1);
                telemetry.addData("Motors", "Pushing");

            } else {
                telemetry.addData("Motors", "Idle");
                leftManipulator.setPower(0);
                rightManipulator.setPower(0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.update();
            sleep(25);
        }
    }
}



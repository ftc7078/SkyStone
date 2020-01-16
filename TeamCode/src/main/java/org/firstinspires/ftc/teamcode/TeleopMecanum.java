

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.sqrt;


@TeleOp(name="Teleop Mecanum", group="Linear Opmode")

public class TeleopMecanum extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor rightManipulator = null;
    private DcMotor leftManipulator = null;
    private final int FL=0;
    private final int FR=1;
    private final int BL =2;
    private final int BR =3;
    private final double MSPEED=1.0;
    private final double SLOW=0.4;
    private final double MAX_SPEED=2800;
    private MecanumDrive mecanumDrive = new MecanumDrive();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        mecanumDrive.init(hardwareMap, telemetry, this);



        leftManipulator = hardwareMap.get(DcMotor.class, "left_manipulator");
        rightManipulator = hardwareMap.get(DcMotor.class, "right_manipulator");


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

            double speed = 1;
             if ( gamepad1.left_trigger > 0.5 ) {
                 speed = 0.6;
             }
             if ( gamepad1.right_trigger > 0.5) {
                 speed = 0.3;
             }

            mecanumDrive.setMotors(gamepad1.left_stick_x,gamepad1.left_stick_y, gamepad1.right_stick_x, speed);


            boolean pull = gamepad2.b;
            boolean push = gamepad2.x;
            if (pull) {
                leftManipulator.setPower(MSPEED);
                rightManipulator.setPower(-MSPEED);
                telemetry.addData("Manipulator Motors", "Pulling");
            } else if (push) {
                leftManipulator.setPower(-MSPEED);
                rightManipulator.setPower(MSPEED);
                telemetry.addData("Manipulator Motors", "Pushing");
            } else {
                telemetry.addData("Manipulator Motors", "Idle");
                leftManipulator.setPower(0);
                rightManipulator.setPower(0);
            }

            mecanumDrive.tickSleep();
            telemetry.addData("Left/Right Stick", "LX (%.2f), LY (%.2f), RX (%.2f), RY (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);

            telemetry.update();
        }
    }

}
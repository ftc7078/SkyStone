

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Teleop Mecanum", group="Linear Opmode")

public class TeleopMecanum extends LinearOpMode {

    private AtlasRobot robot = new AtlasRobot();
    private ElapsedTime runtime = new ElapsedTime();

    /*
    private DcMotor rightManipulator = null;
    //private DcMotor leftManipulator = null;
    private final int FL=0;
    private final int FR=1;
    private final int BL =2;
    private final int BR =3;
    private final double SLOW=0.4;
    private final double MAX_SPEED=2800;
    Servo capstone;
    Servo foundationRight;
    Servo foundationLeft;
    double capstonePosition=0;
    private double mSpeed=1.0;

     */

    private MecanumDrive mecanumDrive = new MecanumDrive();

    @Override
    public void runOpMode() {
        mecanumDrive.init(hardwareMap, telemetry, this);
        robot.init(hardwareMap, telemetry, this);



        //capstone = hardwareMap.get(Servo.class, "capstone");
        //foundationRight = hardwareMap.get(Servo.class, "foundationRight");
        //foundationLeft = hardwareMap.get(Servo.class, "foundationLeft");



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
             //if ( gamepad1.left_trigger > 0.5 ) {
             //    speed = 0.6;
             //}
             //if ( gamepad1.right_trigger > 0.5) {
             //    speed = 0.3;
             //}
             speed = (gamepad1.right_trigger * 0.7) + 0.3;
            double fwd = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rot= gamepad1.right_stick_x;

            fwd = fwd * speed;
            strafe =strafe * speed * 2;
            if (strafe > 1) {
                strafe = 1;
            } else if (strafe < -1) {
                strafe = -1;
            }
            rot = rot * speed;
            mecanumDrive.setMotors(strafe,fwd,rot, 1);

            boolean pull = gamepad2.b;
            boolean push = gamepad2.x;

            if (gamepad2.left_bumper && gamepad2.right_bumper) {
                robot.setCapstone(AtlasRobot.CapstonePosition.DOWN);
            } else if (gamepad2.left_bumper || gamepad2.right_bumper) {
                robot.setCapstone(AtlasRobot.CapstonePosition.MIDDLE);
            } else {
                robot.setCapstone(AtlasRobot.CapstonePosition.UP);
            }
            if (pull) {
                robot.setManipulator(AtlasRobot.ManipulatorDirection.IN);
                telemetry.addData("Manipulator Motors", "Pulling");
            } else if (push) {
                robot.setManipulator(AtlasRobot.ManipulatorDirection.OUT);
                telemetry.addData("Manipulator Motors", "Pushing");
            } else {
                telemetry.addData("Manipulator Motors", "Idle");
                robot.setManipulator(AtlasRobot.ManipulatorDirection.STOP);
            }
            if ( gamepad2.y){
                robot.foundationMover(false);
            }else {
                robot.foundationMover(true);
            }

            mecanumDrive.tickSleep();
            telemetry.addData("Left/Right Stick", "LX (%.2f), LY (%.2f), RX (%.2f), RY (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);

            telemetry.update();
        }

    }

}

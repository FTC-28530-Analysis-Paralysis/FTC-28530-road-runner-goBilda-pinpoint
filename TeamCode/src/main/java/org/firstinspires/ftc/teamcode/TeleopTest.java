package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class TeleopTest extends OpMode {

    // Create a org.firstinspires.ftc.teamcode.RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this, telemetry);

    double forward;
    double turn;
    double strafe;
    double brake = 0, aLastTime = 0, bLastTime = 0, xLastTime = 0, yLastTime = 0, rBLastTime = 0, lBLastTime = 0, dPadUpLastTime = 0, dpadDownLastTime = 0, rightTriggerLastTime = 0, leftTriggerLastTime = 0;
    boolean aButtonPressed = false, bButtonPressed = false, xButtonPressed = false, yButtonPressed = false, dPadUpPressed = false, dPadDownPressed = false, dPadLeftPressed = false, dPadRightPressed = false,
            rightTriggerPressed = false, leftTriggerPressed = false, backButtonPressed = false, startButtonPressed = false, preHangStarted = false, hangStarted = false;
    final double BUTTON_PRESS_DELAY = .075;// seconds, keep track of how long a button has been pressed and allow for a quick press to move a servo a small amount while a long press moves the servo a longer distance.

    ElapsedTime runtime = new ElapsedTime();

    // This method will be called once, when the INIT button is pressed.
    @Override
    public void init() {
        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play to start OpMode.");
        telemetry.update();
    }

    // This method will be called repeatedly during the period between when the init button is pressed and when the play button is pressed (or theOpMode is stopped).
    @Override
    public void init_loop() {

    }

    // This method will be called once, when the play button is pressed.
    @Override
    public void start() {
        runtime.reset();
    }

    // This method will be called repeatedly during the period between when the play button is pressed and when the OpMode is stopped.
    @Override
    public void loop() {
        robot.opModeActive = true;
        robot.updateArmState(); // update arm state machine to track arm position. By calling it here it gets updated everytime the opMode loops but otherwise works in the background while motors move.
        robot.updateLiftState(); // update lift state machine to track lift position.

        // Run wheels in strafer mode (note: The joystick goes negative when pushed forward, so negate it)
        // In this mode the Left stick moves the robot fwd and back and left and right, the Right stick turns left and right.
        // This way it's easy to drive diagonally and have good control of heading.

        /// Mr. Morris: Alternatively we could use right trigger for forward, left trigger for reverse, left_stick_x for strafing and right_stick_x for turning,
        ///             then left/right stick_y could be free for arms or something
        brake = Range.clip(gamepad1.right_stick_y, -1, 0);
        forward = -gamepad1.left_stick_y * (1 - brake);
        strafe = gamepad1.left_stick_x * (1 - brake);
        turn = gamepad1.right_stick_x * (1 - brake);

        // Combine forward and turn for blended motion. Use org.firstinspires.ftc.teamcode.RobotHardware class
        robot.mechanumDrive(forward, strafe, turn);
    }

    // This method will be called once, when this OpMode is stopped.
    @Override
    public void stop() {

    }
}

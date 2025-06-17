/* Copyright (c) 2017 FIRST. All rights reserved.
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

package Skills_USA;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name="Tank Track Control w/ Preset Moves", group="Robot")
//@Disabled
public class MorrisSkillsUSAGeminiPresetMoveEdit extends OpMode{

    /* Declare OpMode members. */

    public DcMotorEx left_front_drive = null;
    public DcMotorEx left_rear_drive = null;
    public DcMotorEx right_front_drive = null;
    public DcMotorEx right_rear_drive = null;
    public DcMotorEx arm_motor = null;
    public DcMotorEx slide_motor = null;
    public Servo wrist = null;
    public Servo   claw   = null;
    private VisionPortal visionPortal;
    // Removed clawOffset as it wasn't used, clawPosition is used directly
    // final double ARM_TICKS_PER_DEGREE = ... // This constant seems specific to an arm design, ensure it's correct for yours if used.

    // --- Constants for Drivetrain ---
    public static final double MOTOR_ENCODER_TICKS_PER_REVOLUTION = 537.7; // For goBILDA 312 RPM
    public static final double MOTOR_MAX_RPM = 312.0;
    public static final double MOTOR_MAX_RPS = MOTOR_MAX_RPM / 60.0;
    public static final double MAX_MOTOR_TICKS_PER_SECOND = MOTOR_MAX_RPS * MOTOR_ENCODER_TICKS_PER_REVOLUTION; // Approx 2796.04

    public static final double TRACK_PITCH_MM = 24.0;
    public static final double TRACK_PITCH_INCHES = TRACK_PITCH_MM / 25.4; // Approx 0.94488
    public static final double TRACK_SPROCKET_TEETH = 12.0;
    public static final double MOTOR_TO_DRIVEN_GEAR_RATIO = 36.0 / 24.0; // 1.5
    public static final double OVERALL_GEAR_RATIO_MOTOR_TO_SPROCKET = MOTOR_TO_DRIVEN_GEAR_RATIO;

    public static final double SPROCKET_CIRCUMFERENCE_INCHES = TRACK_SPROCKET_TEETH * TRACK_PITCH_INCHES; // Approx 11.33856
    public static final double MOTOR_TICKS_PER_SPROCKET_REV = MOTOR_ENCODER_TICKS_PER_REVOLUTION * OVERALL_GEAR_RATIO_MOTOR_TO_SPROCKET; // Approx 806.55

    public static final double TICKS_PER_INCH = MOTOR_TICKS_PER_SPROCKET_REV / SPROCKET_CIRCUMFERENCE_INCHES; // Approx 71.1329
    public static final double TRACK_WIDTH_INCHES = 12.9931; // From (17 + 7/8) - (124 / 25.4)
    public static final double TICKS_PER_DEGREE_ONE_TRACK = calculateTicksPerDegree(); // Approx 4.0326

    // --- Constants for Preset Drivetrain Moves ---
    public static final double PRESET_MOVE_SPEED = 0.6; // Speed for RUN_TO_POSITION (0.0 to 1.0)
    public static final double PRESET_TURN_SPEED = 0.4; // Speed for RUN_TO_POSITION (0.0 to 1.0)
    public static final double PRESET_MOVE_DISTANCE = 6; // Distance for RUN_TO_POSITION (in inches)
    public static final double PRESET_TURN_ANGLE = 22.5; // Angle for RUN_TO_POSITION (in degrees)

    private static final boolean USE_WEBCAM = true;
    public static final double CLAW_SPEED  = 0.03 ; // Not used, direct calculation from trigger
    public static final double WRIST_SPEED = 0.01 ;   // Adjusted for RUN_TO_POSITION
    public static final double ARM_POWER =  0.75 ;   // Adjusted for RUN_TO_POSITION
    public static final double SLIDE_POWER = 0.75 ;   // Adjusted for RUN_TO_POSITION
    public static final int ARM_INCREMENT = 5; // Adjusted for smoother control with right_stick_y
    public static int armTargetPos;
    // public static final int ARM_MIN = 0; // Your commented out limits
    // public static final int ARM_MAX = 2000; // Example, set your actual max
    public static final int SLIDE_INCREMENT = 30; // Adjusted for smoother control
    public static int slideTargetPos; // Added for slide motor
    // public static final int SLIDE_MAX = 1000; // Your commented out limits
    // public static final int SLIDE_MIN = 0;    // Your commented out limits

    public static double clawPosition;
    public static final double CLAW_STOWED = 0.55; // Initial stowed position
    public static double wristPosition;
    public static final double WRIST_FOLDED = 0.28; // Initial folded position

    public static boolean turtleMode = false;
    public static boolean gamepad1_back_pressed_last_frame = false; // For turtle mode toggle
    public static final double TURTLE_MODE_SPEED = .2;
    public static double driveSpeed = 1.0; // Current drive speed multiplier

    // --- Constants for Arm/Wrist Presets ---
    public static final int ARM_PRESET_HIGH_TICKS = 900; //
    public static final double WRIST_PRESET_HIGH_POS = 1.0; // Tipped up // TODO: Placeholder - adjust after testing

    public static final int ARM_PRESET_MIDDLE_TICKS = 450; /// TODO: Placeholder - adjust after testing
    public static final double WRIST_PRESET_MIDDLE_POS = 0.75; // Tipped forward // TODO: Placeholder - adjust after testing

    public static final int ARM_PRESET_LOW_TICKS = 100;  /// TODO: Placeholder - adjust after testing

    // Add boolean flags for button presses (edge detection)
    private boolean gamepad1_y_pressed_last_frame = false;
    private boolean gamepad1_x_pressed_last_frame = false;
    private boolean gamepad1_a_pressed_last_frame = false;

    // --- D-Pad Button States for Preset Moves ---
    private boolean gamepad1_dpad_up_pressed_last_frame = false;
    private boolean gamepad1_dpad_down_pressed_last_frame = false;
    private boolean gamepad1_dpad_left_pressed_last_frame = false;
    private boolean gamepad1_dpad_right_pressed_last_frame = false;

    // --- Joystick Override Threshold ---
    private static final double JOYSTICK_THRESHOLD = 0.1; // Minimum joystick input to be considered active for override

    // Helper to calculate Ticks Per Degree for robot turning
    private static double calculateTicksPerDegree() {
        if (TRACK_WIDTH_INCHES <= 0 || TICKS_PER_INCH <= 0) {
            // Prevent division by zero or nonsensical values if constants aren't set properly
            return 0;
        }
        // The distance each track needs to travel for the robot to turn a certain angle.
        // For a 360-degree turn of the robot, the center of each track travels
        // along a circle. The radius of this circle is TRACK_WIDTH_INCHES / 2.
        // The circumference of this circle is PI * TRACK_WIDTH_INCHES.
        // This is the distance one track travels *relative to the other* for a 360-degree turn.
        // So, one track moves forward by (PI * TRACK_WIDTH_INCHES) / 2
        // and the other moves backward by (PI * TRACK_WIDTH_INCHES) / 2.
        // The absolute distance covered by one track for a 360-degree robot turn is (PI * TRACK_WIDTH_INCHES) / 2.

        double distanceOneTrackTravelsFor360Turn = (Math.PI * TRACK_WIDTH_INCHES) / 2.0;
        double ticksFor360TurnOneTrack = distanceOneTrackTravelsFor360Turn * TICKS_PER_INCH;
        return ticksFor360TurnOneTrack / 360.0; // Ticks per degree for one track
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        // visionPortal = builder.build(); // Initialize if you are using vision features

        // Define and Initialize Motors
        left_front_drive = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        left_rear_drive = hardwareMap.get(DcMotorEx.class, "left_rear_drive");
        right_front_drive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        right_rear_drive = hardwareMap.get(DcMotorEx.class, "right_rear_drive");
        arm_motor = hardwareMap.get(DcMotorEx.class, "lift");
        slide_motor = hardwareMap.get(DcMotorEx.class, "slide");

        // Set motor directions (IMPORTANT: Test and adjust these for your specific robot)
        left_front_drive.setDirection(DcMotorSimple.Direction.FORWARD); // Your setting
        left_rear_drive.setDirection(DcMotorSimple.Direction.FORWARD);  // Your setting
        right_front_drive.setDirection(DcMotorSimple.Direction.REVERSE);// Your setting
        right_rear_drive.setDirection(DcMotorSimple.Direction.REVERSE); // Your setting
        arm_motor.setDirection(DcMotorSimple.Direction.FORWARD);    // Your setting
        slide_motor.setDirection(DcMotorSimple.Direction.REVERSE);  // Your setting

        // Set Zero Power Behavior to BRAKE for all motors
        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_rear_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_rear_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set Drive Motors to RUN_USING_ENCODER for TeleOp velocity control by default
        // and reset encoders.
        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Arm and Slide motor setup
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Arm and slide will be set to RUN_TO_POSITION in the loop as per your original logic
        // but initialize their target positions.
        armTargetPos = arm_motor.getCurrentPosition();
        slideTargetPos = slide_motor.getCurrentPosition(); // Initialize slideTargetPos

        // Define and initialize servos.
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        // Set initial servo positions
        wristPosition = WRIST_FOLDED;
        clawPosition = CLAW_STOWED;
        wrist.setPosition(wristPosition);
        claw.setPosition(clawPosition);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready. Press START.");
        telemetry.addData("Ticks/Inch", "%.2f", TICKS_PER_INCH);
        telemetry.addData("Ticks/Degree", "%.4f", calculateTicksPerDegree());
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // You can add telemetry here if needed during init_loop
        // e.g., to confirm sensor readings before start
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        // Code here will run once when play is pressed.
        // Could reset timers or other game-start specific logic.
        // Ensure arm and slide target positions are current if they could have drifted.
        armTargetPos = arm_motor.getCurrentPosition();
        slideTargetPos = slide_motor.getCurrentPosition();
    }
    // --- Helper Methods for Drivetrain Control ---

    /**
     * Sets the mode for all drive motors.
     * @param runMode The DcMotor.RunMode to set.
     */
    private void setDriveMotorMode(DcMotor.RunMode runMode) {
        left_front_drive.setMode(runMode);
        left_rear_drive.setMode(runMode);
        right_front_drive.setMode(runMode);
        right_rear_drive.setMode(runMode);
    }

    /**
     * Sets the power for all drive motors.
     * Useful for stopping motors or setting a common power during RUN_TO_POSITION.
     * @param power The power level (-1.0 to 1.0).
     */
    private void setDriveMotorPower(double power) {
        left_front_drive.setPower(power);
        left_rear_drive.setPower(power);
        right_front_drive.setPower(power);
        right_rear_drive.setPower(power);
    }

    /**
     * Moves the robot forward or backward a specific distance.
     * This method initiates the move; completion is handled by checking motor.isBusy().
     * Joystick input can interrupt this move.
     * @param distanceInches The distance to move in inches (positive for forward, negative for backward).
     */
    private void moveForwardDistance(double distanceInches) {

        int targetTicks = (int) (distanceInches * TICKS_PER_INCH);

        // Reset encoders before setting new target for relative movement
        // This ensures the move is relative to the current position.
        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_front_drive.setTargetPosition(targetTicks);
        left_rear_drive.setTargetPosition(targetTicks);
        right_front_drive.setTargetPosition(targetTicks);
        right_rear_drive.setTargetPosition(targetTicks);

        // Set mode to RUN_TO_POSITION
        setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power for the move
        setDriveMotorPower(PRESET_MOVE_SPEED);
        telemetry.addData("Preset Move", "Forward: %.2f inches", distanceInches);
    }

    /**
     * Turns the robot a specific angle.
     * This method initiates the turn; completion is handled by checking motor.isBusy().
     * Joystick input can interrupt this move.
     * @param angleInDegrees The angle to turn in degrees (positive for left/counter-clockwise, negative for right/clockwise).
     */
    private void turnAngle(double angleInDegrees) {

        // For a point turn, one side moves forward, the other backward.
        // The TICKS_PER_DEGREE_ONE_TRACK should represent the ticks one track needs to move
        // for one degree of robot rotation.
        int turnTicksOneTrack = (int) (angleInDegrees * calculateTicksPerDegree());

        // Reset encoders before setting new target for relative movement
        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Positive angle = turn left (left tracks reverse, right tracks forward)
        // Negative angle = turn right (left tracks forward, right tracks reverse)
        left_front_drive.setTargetPosition(-turnTicksOneTrack);
        left_rear_drive.setTargetPosition(-turnTicksOneTrack);
        right_front_drive.setTargetPosition(turnTicksOneTrack);
        right_rear_drive.setTargetPosition(turnTicksOneTrack);

        // Set mode to RUN_TO_POSITION
        setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power for the turn
        setDriveMotorPower(PRESET_TURN_SPEED);
        telemetry.addData("Preset Move", "Turn: %.2f degrees", angleInDegrees);
    }
    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        double joystickDriveInput = -gamepad1.left_stick_y; // Negative because joystick forward is negative
        double joystickTurnInput = -gamepad1.left_stick_x;  // Standard arcade: left stick X for turn

        boolean significantJoystickInput = Math.abs(joystickDriveInput) > JOYSTICK_THRESHOLD ||
                Math.abs(joystickTurnInput) > JOYSTICK_THRESHOLD;

        // --- Turtle Mode Logic ---
        if (gamepad1.back && !gamepad1_back_pressed_last_frame) {
            gamepad1_back_pressed_last_frame = true; // Latch: Record that the button was pressed
        } else if (!gamepad1.back && gamepad1_back_pressed_last_frame) {
            gamepad1_back_pressed_last_frame = false; // Latch: Button released, now toggle the mode
            turtleMode = !turtleMode;
        }
        driveSpeed = turtleMode ? TURTLE_MODE_SPEED : 1.0;

        // --- Arm and Wrist Preset Logic ---
        boolean armPresetActivatedThisLoop = false;

        // Y Button: High Position
        if (gamepad1.y && !gamepad1_y_pressed_last_frame) {
            armTargetPos = ARM_PRESET_HIGH_TICKS;
            wristPosition = WRIST_PRESET_HIGH_POS;
            armPresetActivatedThisLoop = true;
            telemetry.addData("Preset", "Arm/Wrist HIGH");
        }
        gamepad1_y_pressed_last_frame = gamepad1.y;

        // X Button: Middle Position
        if (gamepad1.x && !gamepad1_x_pressed_last_frame) {
            armTargetPos = ARM_PRESET_MIDDLE_TICKS;
            wristPosition = WRIST_PRESET_MIDDLE_POS;
            armPresetActivatedThisLoop = true;
            telemetry.addData("Preset", "Arm/Wrist MIDDLE");
        }
        gamepad1_x_pressed_last_frame = gamepad1.x;

        // A Button: Low Position
        if (gamepad1.a && !gamepad1_a_pressed_last_frame) {
            armTargetPos = ARM_PRESET_LOW_TICKS;
            wristPosition = WRIST_PRESET_MIDDLE_POS;
            armPresetActivatedThisLoop = true;
            telemetry.addData("Preset", "Arm/Wrist LOW");
        }
        gamepad1_a_pressed_last_frame = gamepad1.a;

        // --- Primary Drive Control Logic ---
        if (significantJoystickInput) {
            // If joystick is active, it overrides any preset move.
            if (left_front_drive.isBusy() || right_front_drive.isBusy() ||
                    left_rear_drive.isBusy() || right_rear_drive.isBusy()) {
                // A preset move was active, stop it immediately.
                setDriveMotorPower(0.0); // Stop all motors
                // Mode will be set to RUN_USING_ENCODER below.
            }
            // Ensure motors are in RUN_USING_ENCODER for joystick velocity control
            if (left_front_drive.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // --- Joystick Arcade Drive with Velocity Control ---
            // Your existing arcade drive logic:
            double leftPowerRaw = joystickDriveInput - joystickTurnInput; // Corrected: was -gamepad1.left_stick_x
            double rightPowerRaw = joystickDriveInput + joystickTurnInput; // Corrected: was -gamepad1.left_stick_x

            double denominator = Math.max(Math.abs(joystickDriveInput) + Math.abs(joystickTurnInput), 1.0);
            double leftScaledPower = leftPowerRaw / denominator;
            double rightScaledPower = rightPowerRaw / denominator;

            double finalLeftPower = leftScaledPower * driveSpeed; // Apply turtle mode
            double finalRightPower = rightScaledPower * driveSpeed; // Apply turtle mode

            double leftTargetVelocity = finalLeftPower * MAX_MOTOR_TICKS_PER_SECOND;
            double rightTargetVelocity = finalRightPower * MAX_MOTOR_TICKS_PER_SECOND;

            left_front_drive.setVelocity(leftTargetVelocity);
            left_rear_drive.setVelocity(leftTargetVelocity);
            right_front_drive.setVelocity(rightTargetVelocity);
            right_rear_drive.setVelocity(rightTargetVelocity);

            telemetry.addData("Control", "Joystick Active");
            telemetry.addData("Drive Input Raw", "%.2f", joystickDriveInput);
            telemetry.addData("Turn Input Raw", "%.2f", joystickTurnInput);
            // telemetry.addData("Left Target TPS", "%.2f", leftTargetVelocity);
            // telemetry.addData("Right Target TPS", "%.2f", rightTargetVelocity);

        } else {
            // --- No significant joystick input: Process D-Pad Presets or let ongoing preset continue ---
            boolean presetMoveIsActive = left_front_drive.isBusy() || right_front_drive.isBusy() ||
                    left_rear_drive.isBusy() || right_rear_drive.isBusy();

            if (presetMoveIsActive) {
                // A preset move is running. Motors are in RUN_TO_POSITION.
                // The helper methods (moveForwardDistance, turnAngle) have already set the power.
                // We just let them continue until they are no longer busy.
                // Telemetry for the active preset move is handled in the helper methods when initiated.
                telemetry.addData("Status", "Preset Move Executing");
            } else {
                // No preset move is active, and no joystick input.
                // Ensure motors are in RUN_USING_ENCODER and stopped if they were previously in RUN_TO_POSITION.
                if (left_front_drive.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    setDriveMotorPower(0.0); // Ensure motors are stopped after a preset finishes
                }
                // If already in RUN_USING_ENCODER, ensure power is zero since there's no joystick input.
                // This prevents drift if joystick was released quickly.
                else if (left_front_drive.getPower() != 0 || right_front_drive.getPower() != 0) {
                    setDriveMotorPower(0.0);
                }


                telemetry.addData("Control", "Awaiting D-Pad or Joystick");

                // D-pad Up: Move forward 6 inches
                if (gamepad1.dpad_up && !gamepad1_dpad_up_pressed_last_frame) {
                    moveForwardDistance(6.0);
                }
                gamepad1_dpad_up_pressed_last_frame = gamepad1.dpad_up;

                // D-pad Down: Move backward 6 inches
                if (gamepad1.dpad_down && !gamepad1_dpad_down_pressed_last_frame) {
                    moveForwardDistance(-6.0);
                }
                gamepad1_dpad_down_pressed_last_frame = gamepad1.dpad_down;

                // D-pad Right: Turn right 22.5 degrees
                if (gamepad1.dpad_right && !gamepad1_dpad_right_pressed_last_frame) {
                    turnAngle(-PRESET_TURN_ANGLE); // Negative angle for turning right
                }
                gamepad1_dpad_right_pressed_last_frame = gamepad1.dpad_right;

                // D-pad Left: Turn left 22.5 degrees
                if (gamepad1.dpad_left && !gamepad1_dpad_left_pressed_last_frame) {
                    turnAngle(PRESET_TURN_ANGLE); // Positive angle for turning left
                }
                gamepad1_dpad_left_pressed_last_frame = gamepad1.dpad_left;
            }
        }

        // --- Claw and Wrist Control ---
        clawPosition += (gamepad1.right_trigger - gamepad1.left_trigger) * CLAW_SPEED;
        // Claw limit
        if (clawPosition > 1) clawPosition = 1;
        else if (clawPosition < 0) clawPosition = 0;
        claw.setPosition(clawPosition);

        if (!armPresetActivatedThisLoop) { // Only allow manual wrist if no arm/wrist preset was just hit
            if (gamepad1.right_bumper) wristPosition += WRIST_SPEED;
            else if (gamepad1.left_bumper) wristPosition -= WRIST_SPEED;
        }
        //Wrist limit (apply regardless of preset or manual)
        if (wristPosition > 1) wristPosition = 1;
        else if (wristPosition < 0) wristPosition = 0;
        wrist.setPosition(wristPosition); // Set wrist position

        // --- Arm and Slide Control (Your Existing Logic for continuous control) ---
        // Adjust target positions based on right stick
        if (!armPresetActivatedThisLoop) {
            armTargetPos = armTargetPos - (int) (gamepad1.right_stick_y * ARM_INCREMENT);
        }
        slideTargetPos = slideTargetPos + (int) (gamepad1.right_stick_x * SLIDE_INCREMENT);

        // Apply limits (optional, if you define ARM_MIN/MAX, SLIDE_MIN/MAX)
        // if (armTargetPos > ARM_MAX) armTargetPos = ARM_MAX;
        // if (armTargetPos < ARM_MIN) armTargetPos = ARM_MIN;
        // if (slideTargetPos > SLIDE_MAX) slideTargetPos = SLIDE_MAX;
        // if (slideTargetPos < SLIDE_MIN) slideTargetPos = SLIDE_MIN;

        arm_motor.setTargetPosition(armTargetPos);
        slide_motor.setTargetPosition(slideTargetPos);

        // Set mode and power for arm and slide (always active for RUN_TO_POSITION)
        // Ensure the motors are in the correct mode if they aren't already.
        if (arm_motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (slide_motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            slide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        // Set power for arm and slide. Power should be positive as direction is handled by target position.
        arm_motor.setPower(ARM_POWER); // Adjust power as needed for arm responsiveness
        slide_motor.setPower(SLIDE_POWER); // Adjust power as needed for slide responsiveness


        // --- Telemetry ---
        // Drive telemetry is handled within the if/else blocks for joystick/preset
        telemetry.addData("Turtle Mode", turtleMode ? "ON" : "OFF");
        telemetry.addData("Drive Speed Multiplier", "%.2f", driveSpeed);
        telemetry.addData("Claw Position", "%.2f", clawPosition);
        telemetry.addData("Wrist Position", "%.2f", wristPosition);
        telemetry.addData("Arm Target Pos", armTargetPos);
        telemetry.addData("Arm Current Pos", arm_motor.getCurrentPosition());
        telemetry.addData("Slide Target Pos", slideTargetPos);
        telemetry.addData("Slide Current Pos", slide_motor.getCurrentPosition());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Stop all motors and release resources
        setDriveMotorPower(0);
        arm_motor.setPower(0);
        slide_motor.setPower(0);

        // Optional: Set motors to a specific mode on stop if desired, e.g., COAST
        // setDriveMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Or BRAKE
        // arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // slide_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (visionPortal != null) {
            visionPortal.close();
        }
        telemetry.addData(">", "Robot Stopped.");
        telemetry.update();
    }
}
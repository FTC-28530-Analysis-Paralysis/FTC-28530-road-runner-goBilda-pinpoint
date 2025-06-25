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

@TeleOp(name="Atlanta Preset Moves", group="Robot")
//@Disabled
public class AtlantaGeminiPresetMoves extends OpMode{

//--------------------------------------------------------------------------------------------------
// OpMode Class Members
//--------------------------------------------------------------------------------------------------

  // --- Hardware Device Declarations ---
    // Drivetrain Motors
    public DcMotorEx left_front_drive = null;
    public DcMotorEx left_rear_drive = null;
    public DcMotorEx right_front_drive = null;
    public DcMotorEx right_rear_drive = null;

    // Mechanism Motors
    public DcMotorEx arm_motor = null;
    public DcMotorEx slide_motor = null;

    // Servos
    public Servo wrist = null;
    public Servo claw = null;

    // Vision System
    private VisionPortal visionPortal; // Initialized in init() if USE_WEBCAM is true
    private static final boolean USE_WEBCAM = true; // Master switch for webcam initialization

//--------------------------------------------------------------------------------------------------
// Drivetrain Constants & Variables
//--------------------------------------------------------------------------------------------------

  // --- Physical Properties & Ratios (Tune these for your specific robot build) ---
    public static final double MOTOR_ENCODER_TICKS_PER_REVOLUTION = 537.7; // For goBILDA 5203 Series Yellow Jacket Motor (312 RPM)
    public static final double MOTOR_MAX_RPM = 312.0;
    // Derived motor speed values
    public static final double MOTOR_MAX_RPS = MOTOR_MAX_RPM / 60.0; // Revolutions per second
    public static final double MAX_MOTOR_TICKS_PER_SECOND = MOTOR_MAX_RPS * MOTOR_ENCODER_TICKS_PER_REVOLUTION; // Max ticks per second for velocity control

    public static final double TRACK_PITCH_MM = 24.0; // Pitch of the tank track links in millimeters
    public static final double TRACK_PITCH_INCHES = TRACK_PITCH_MM / 25.4; // Track pitch in inches
    public static final double TRACK_SPROCKET_TEETH = 12.0; // Number of teeth on the drive sprocket
    public static final double MOTOR_TO_DRIVEN_GEAR_RATIO = 36.0 / 24.0; // Example: 36-tooth motor gear, 24-tooth sprocket gear
    public static final double OVERALL_GEAR_RATIO_MOTOR_TO_SPROCKET = MOTOR_TO_DRIVEN_GEAR_RATIO; // Overall gear reduction from motor to sprocket

    // --- Calculated Drivetrain Metrics (Based on physical properties) ---
    public static final double SPROCKET_CIRCUMFERENCE_INCHES = TRACK_SPROCKET_TEETH * TRACK_PITCH_INCHES; // Circumference of the drive sprocket
    public static final double MOTOR_TICKS_PER_SPROCKET_REV = MOTOR_ENCODER_TICKS_PER_REVOLUTION * OVERALL_GEAR_RATIO_MOTOR_TO_SPROCKET;
    public static final double TICKS_PER_INCH = MOTOR_TICKS_PER_SPROCKET_REV / SPROCKET_CIRCUMFERENCE_INCHES; // Encoder ticks per inch of robot travel

    public static final double TRACK_WIDTH_INCHES = 12.9931; // Effective distance between the center of the left and right tracks
                                                             // From total robot width estimated at 17 7/8 inches - one track width of 124 mm
    public static final double TICKS_PER_DEGREE_ONE_TRACK = calculateTicksPerDegree(); // Calculated in the helper method, can be pre-calculated if preferred

    // --- Drivetrain Control Parameters ---
    public static final double PRESET_MOVE_SPEED = 0.6;     // Default power/speed for preset forward/backward movements (0.0 to 1.0)
    public static final double PRESET_TURN_SPEED = 0.4;     // Default power/speed for preset turns (0.0 to 1.0)
    public static final double PRESET_MOVE_DISTANCE = 6.0;  // Default distance for d-pad forward/backward preset (inches)
    public static final double PRESET_TURN_ANGLE = 22.5;    // Default angle for d-pad turn preset (degrees) TODO: Test by setting this angle much higher, say 720 degrees.

    public static final double TURN_CALIBRATION_MULTIPLIER = 1.0 * (720.0 / 620.0); // Start at 1.0. This adjustment is to be made after empirical testing to account for track slipping, scrubbing, etc.
    // TODO: Increase if robot under-turns.
    // Decrease if robot over-turns.
    // Example: If robot turns 330deg when 360deg is commanded,
    // new_multiplier = old_multiplier * (360.0 / 330.0)

    public static boolean turtleMode = false;               // State variable for turtle mode (reduced speed)
    public static final double TURTLE_MODE_SPEED = 0.2;     // Speed multiplier when turtle mode is active
    public static double driveSpeed = 1.0;                  // Current drive speed multiplier (affected by turtle mode)

//--------------------------------------------------------------------------------------------------
// Mechanism Constants & Variables (Arm, Slide, Claw, Wrist)
//--------------------------------------------------------------------------------------------------

    // --- Arm Control ---
    public static final double ARM_RTP_MAX_SPEED = 0.75;    // Max speed/power for arm during RUN_TO_POSITION
    public static final int ARM_MANUAL_INCREMENT = 15;      // Ticks to increment/decrement arm target per joystick input cycle
    public static int armTargetPos = 0;                     // Target position for the arm motor (in encoder ticks)
// public static final int ARM_MIN_TICKS = 0;           // Optional: Minimum allowed arm encoder ticks
// public static final int ARM_MAX_TICKS = 2000;        // Optional: Maximum allowed arm encoder ticks

    // --- Slide Control ---
    public static final double SLIDE_RTP_MAX_SPEED = 0.75;  // Max speed/power for slide during RUN_TO_POSITION
    public static final int SLIDE_MANUAL_INCREMENT = 30;    // Ticks to increment/decrement slide target per joystick input cycle
    public static int slideTargetPos = 0;                   // Target position for the slide motor (in encoder ticks)
    // public static final int SLIDE_MIN_TICKS = 0;         // Optional: Minimum allowed slide encoder ticks
    // public static final int SLIDE_MAX_TICKS = 1000;      // Optional: Maximum allowed slide encoder ticks

    // --- Wrist Control ---
    public static final double WRIST_MANUAL_INCREMENT = 0.01; // Servo position change per bumper press cycle
    public static double wristPosition = 0.0;               // Target position for the wrist servo (0.0 to 1.0)
    public static final double WRIST_FOLDED_POS = 0;     // Initial/stowed position for the wrist

    // --- Claw Control ---
    public static final double CLAW_MANUAL_INCREMENT = 0.03; // Servo position change based on trigger input
    public static double clawPosition = 0.0;                // Target position for the claw servo (0.0 to 1.0)
    public static final double CLAW_STOWED_POS = 0;      // Initial/stowed position for the claw

    // --- Arm & Wrist Preset Positions ---
    // High Preset - Tag mailbox with arm and wrist tipped up so the robot can just drive forward until it makes contact
    public static final int ARM_PRESET_HIGH_TICKS = 900;
    public static final int SLIDE_PRESET_HIGH_TICKS = 0;
    public static final double WRIST_PRESET_HIGH_POS = 1.0;    // Wrist tipped up

    // Middle Preset - Lift ordnance off the ground and hold it high enough to be deposited in containment box
    public static final int ARM_PRESET_MIDDLE_TICKS = 52;    // TODO: Placeholder - adjust after testing
    public static final int SLIDE_PRESET_MIDDLE_TICKS = 0;
    public static final double WRIST_PRESET_MIDDLE_POS = WRIST_PRESET_HIGH_POS;

    // Low Preset / Intake Preset - Move arm and wrist next to ground to pick up ordnance
    public static final int ARM_PRESET_LOW_TICKS = -315;       // TODO: Placeholder - adjust after testing
    public static final int SLIDE_PRESET_LOW_TICKS = 345;
    public static final double WRIST_PRESET_LOW_POS = WRIST_PRESET_HIGH_POS;

//--------------------------------------------------------------------------------------------------
// Gamepad Button State Variables (for edge detection)
//--------------------------------------------------------------------------------------------------
    // These flags help detect single button presses rather than continuous holding.

    // --- Gamepad 1 ---
    // Drivetrain Presets
    private boolean gamepad1_dpad_up_pressed_last_frame = false;
    private boolean gamepad1_dpad_down_pressed_last_frame = false;
    private boolean gamepad1_dpad_left_pressed_last_frame = false;
    private boolean gamepad1_dpad_right_pressed_last_frame = false;

    // Arm/Wrist Presets
    private boolean gamepad1_y_pressed_last_frame = false;
    private boolean gamepad1_x_pressed_last_frame = false;
    private boolean gamepad1_a_pressed_last_frame = false;

    // Turtle Mode Toggle
    private boolean gamepad1_back_pressed_last_frame = false;

    // --- Joystick Override Threshold ---
    private static final double JOYSTICK_THRESHOLD = 0.1; // Minimum joystick input to be considered active for override

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
        wristPosition = WRIST_FOLDED_POS;
        clawPosition = CLAW_STOWED_POS;
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

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // --- Get Joystick Inputs for Drive ---
        double joystickDriveInput = -gamepad1.left_stick_y; // Negative because joystick forward is negative
        double joystickTurnInput = -gamepad1.left_stick_x;  // Standard arcade: left stick X for turn

        boolean significantJoystickInput = Math.abs(joystickDriveInput) > JOYSTICK_THRESHOLD ||
                Math.abs(joystickTurnInput) > JOYSTICK_THRESHOLD;

        // --- Turtle Mode Latching & Speed Setting ---
        if (gamepad1.back && !gamepad1_back_pressed_last_frame) {
            // This is the rising edge of the button press
        } else if (!gamepad1.back && gamepad1_back_pressed_last_frame) {
            // This is the falling edge (release), toggle the mode
            turtleMode = !turtleMode;
        }
        gamepad1_back_pressed_last_frame = gamepad1.back; // Update last state
        driveSpeed = turtleMode ? TURTLE_MODE_SPEED : 1.0;


        // --- D-Pad Button Latching for Drive Presets ---
        boolean dpadUpTriggeredThisLoop = false;
        if (gamepad1.dpad_up && !gamepad1_dpad_up_pressed_last_frame) {
            dpadUpTriggeredThisLoop = true;
        }
        gamepad1_dpad_up_pressed_last_frame = gamepad1.dpad_up;

        boolean dpadDownTriggeredThisLoop = false;
        if (gamepad1.dpad_down && !gamepad1_dpad_down_pressed_last_frame) {
            dpadDownTriggeredThisLoop = true;
        }
        gamepad1_dpad_down_pressed_last_frame = gamepad1.dpad_down;

        boolean dpadLeftTriggeredThisLoop = false;
        if (gamepad1.dpad_left && !gamepad1_dpad_left_pressed_last_frame) {
            dpadLeftTriggeredThisLoop = true;
        }
        gamepad1_dpad_left_pressed_last_frame = gamepad1.dpad_left;

        boolean dpadRightTriggeredThisLoop = false;
        if (gamepad1.dpad_right && !gamepad1_dpad_right_pressed_last_frame) {
            dpadRightTriggeredThisLoop = true;
        }
        gamepad1_dpad_right_pressed_last_frame = gamepad1.dpad_right;

        boolean dpadDrivePresetActivatedThisLoop = dpadUpTriggeredThisLoop || dpadDownTriggeredThisLoop ||
                dpadLeftTriggeredThisLoop || dpadRightTriggeredThisLoop;

        // --- Primary Drive Control Logic ---
        if (significantJoystickInput) {
            telemetry.addData("Drive Control", "Joystick Active");
            // If motors were in RUN_TO_POSITION (e.g., from a finished D-Pad preset),
            // switch them back to RUN_USING_ENCODER for joystick control.
            if (left_front_drive.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
                // It's good practice to ensure velocities are zeroed before new ones are applied,
                // though the setVelocity calls below will effectively do this.
            }

            // Joystick Arcade Drive with Velocity Control
            double leftPowerRaw = joystickDriveInput - joystickTurnInput;
            double rightPowerRaw = joystickDriveInput + joystickTurnInput;
            double denominator = Math.max(Math.abs(leftPowerRaw), Math.abs(rightPowerRaw)); // Simpler normalization for tank/arcade
            if (denominator > 1.0) { // Normalize only if necessary
                leftPowerRaw /= denominator;
                rightPowerRaw /= denominator;
            }


            double finalLeftPower = leftPowerRaw * driveSpeed;
            double finalRightPower = rightPowerRaw * driveSpeed;

            double leftTargetVelocity = finalLeftPower * MAX_MOTOR_TICKS_PER_SECOND;
            double rightTargetVelocity = finalRightPower * MAX_MOTOR_TICKS_PER_SECOND;

            left_front_drive.setVelocity(leftTargetVelocity);
            left_rear_drive.setVelocity(leftTargetVelocity);
            right_front_drive.setVelocity(rightTargetVelocity);
            right_rear_drive.setVelocity(rightTargetVelocity);

        } else if (dpadDrivePresetActivatedThisLoop) {
            telemetry.addData("Drive Control", "D-Pad Initiated");
            // D-Pad preset was triggered THIS loop cycle.
            // The moveForwardDistance() or turnAngle() methods will handle:
            // 1. Calculating target ticks
            // 2. setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            // 3. motor.setTargetPosition(targetTicks);
            // 4. setDriveMotorPower(SOME_PRESET_SPEED);

            if (dpadUpTriggeredThisLoop) {
                moveForwardDistance(PRESET_MOVE_DISTANCE);
                telemetry.addData("D-Pad Action", "Forward");
            } else if (dpadDownTriggeredThisLoop) {
                moveForwardDistance(-PRESET_MOVE_DISTANCE);
                telemetry.addData("D-Pad Action", "Backward");
            } else if (dpadLeftTriggeredThisLoop) {
                turnAngle(PRESET_TURN_ANGLE); // Positive angle for turning left
                telemetry.addData("D-Pad Action", "Turn Left");
            } else if (dpadRightTriggeredThisLoop) {
                turnAngle(-PRESET_TURN_ANGLE); // Negative angle for turning right
                telemetry.addData("D-Pad Action", "Turn Right");
            }
        } else {
            // No significant joystick input AND no D-Pad preset activated THIS loop.
            // This handles stopping or letting an ONGOING D-Pad move continue.

            boolean presetMoveIsCurrentlyRunning = left_front_drive.isBusy() || right_front_drive.isBusy() ||
                    left_rear_drive.isBusy() || right_rear_drive.isBusy();

            if (presetMoveIsCurrentlyRunning && left_front_drive.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                // A D-Pad initiated preset move is STILL IN PROGRESS from a PREVIOUS loop cycle.
                telemetry.addData("Drive Control", "D-Pad Executing");
                // Do nothing to the motors; let RUN_TO_POSITION complete.
            } else {
                // Robot Should Be STOPPED:
                // - Joystick was released, and no D-Pad preset was pressed this cycle.
                // - A D-Pad preset just finished (areDriveMotorsBusy() is now false).
                // - Robot is idle at start.
                telemetry.addData("Drive Control", "Stopping/Idle");

                // Ensure motors are in RUN_USING_ENCODER and set velocities to zero.
                if (left_front_drive.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                // Explicitly set velocities to zero to stop.
                left_front_drive.setVelocity(0);
                left_rear_drive.setVelocity(0);
                right_front_drive.setVelocity(0);
                right_rear_drive.setVelocity(0);
            }
        }

        // --- Arm and Wrist Preset Logic (using Y, X, A buttons) ---
        boolean armPresetActivatedThisLoop = false; // Reset for arm/wrist manual override logic

        // Y Button: High Position (Latch)
        if (gamepad1.y && !gamepad1_y_pressed_last_frame) {
            armTargetPos = ARM_PRESET_HIGH_TICKS;
            slideTargetPos = SLIDE_PRESET_HIGH_TICKS;
            wristPosition = WRIST_PRESET_HIGH_POS;
            armPresetActivatedThisLoop = true;
            telemetry.addData("Arm/Wrist Preset", "HIGH");
        }
        gamepad1_y_pressed_last_frame = gamepad1.y;

        // X Button: Middle Position (Latch)
        if (gamepad1.x && !gamepad1_x_pressed_last_frame) {
            armTargetPos = ARM_PRESET_MIDDLE_TICKS;
            slideTargetPos = SLIDE_PRESET_MIDDLE_TICKS;
            wristPosition = WRIST_PRESET_MIDDLE_POS;
            armPresetActivatedThisLoop = true;
            telemetry.addData("Arm/Wrist Preset", "MIDDLE");
        }
        gamepad1_x_pressed_last_frame = gamepad1.x;

        // A Button: Low Position (Latch)
        if (gamepad1.a && !gamepad1_a_pressed_last_frame) {
            armTargetPos = ARM_PRESET_LOW_TICKS;
            slideTargetPos = SLIDE_PRESET_LOW_TICKS;
            wristPosition = WRIST_PRESET_LOW_POS;
            armPresetActivatedThisLoop = true;
            telemetry.addData("Arm/Wrist Preset", "LOW");
        }
        gamepad1_a_pressed_last_frame = gamepad1.a;

        // --- Claw Control (Triggers) ---
        clawPosition += (gamepad1.right_trigger - gamepad1.left_trigger) * CLAW_MANUAL_INCREMENT;
        // Claw limits
        if (clawPosition > 1.0) clawPosition = 1.0;
        else if (clawPosition < 0.0) clawPosition = 0.0;
        claw.setPosition(clawPosition);

        // --- Manual Wrist Control (Bumpers) - Overridden by presets for one cycle ---
        if (!armPresetActivatedThisLoop) { // Only allow manual wrist if no arm/wrist preset was just hit
            if (gamepad1.right_bumper) wristPosition += WRIST_MANUAL_INCREMENT;
            else if (gamepad1.left_bumper) wristPosition -= WRIST_MANUAL_INCREMENT;
        }
        // Wrist limits (apply regardless of preset or manual)
        if (wristPosition > 1.0) wristPosition = 1.0;
        else if (wristPosition < 0.0) wristPosition = 0.0;
        wrist.setPosition(wristPosition);

        // --- Manual Arm and Slide Control (Right Stick) - Overridden by presets for one cycle ---
        // Adjust target positions based on right stick
        if (!armPresetActivatedThisLoop) { // Only allow manual arm if no arm/wrist preset was just hit
            // Invert right_stick_y if needed: common for up to be negative
            armTargetPos = armTargetPos - (int) (gamepad1.right_stick_y * ARM_MANUAL_INCREMENT);
        }
        // Slide control is independent of arm presets
        slideTargetPos = slideTargetPos + (int) (gamepad1.right_stick_x * SLIDE_MANUAL_INCREMENT);

        // Apply Arm/Slide Limits (Define ARM_MIN/MAX_TICKS, SLIDE_MIN/MAX_TICKS if needed)
        // if (armTargetPos > ARM_MAX_TICKS) armTargetPos = ARM_MAX_TICKS;
        // if (armTargetPos < ARM_MIN_TICKS) armTargetPos = ARM_MIN_TICKS;
        // if (slideTargetPos > SLIDE_MAX_TICKS) slideTargetPos = SLIDE_MAX_TICKS;
        // if (slideTargetPos < SLIDE_MIN_TICKS) slideTargetPos = SLIDE_MIN_TICKS;

        arm_motor.setTargetPosition(armTargetPos);
        slide_motor.setTargetPosition(slideTargetPos);

        // Set mode and power for arm and slide (always active for RUN_TO_POSITION)
        if (arm_motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (slide_motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            slide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        arm_motor.setPower(ARM_RTP_MAX_SPEED);
        slide_motor.setPower(SLIDE_RTP_MAX_SPEED);

        // --- Telemetry ---
        telemetry.addData("Turtle Mode", turtleMode ? "ON" : "OFF");
        // Drive control telemetry is handled within its specific if/else blocks
        telemetry.addData("Claw Position", "%.2f", clawPosition);
        telemetry.addData("Wrist Position", "%.2f", wristPosition);
        telemetry.addData("Arm Target", armTargetPos);
        telemetry.addData("Arm Current", arm_motor.getCurrentPosition());
        telemetry.addData("Slide Target", slideTargetPos);
        telemetry.addData("Slide Current", slide_motor.getCurrentPosition());
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
        double rawTicksPerDegree = ticksFor360TurnOneTrack / 360.0;

        // Apply the calibration multiplier. This adjusts for any error caused by track slipping and scrubbing, etc.
        return rawTicksPerDegree * TURN_CALIBRATION_MULTIPLIER;
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
}



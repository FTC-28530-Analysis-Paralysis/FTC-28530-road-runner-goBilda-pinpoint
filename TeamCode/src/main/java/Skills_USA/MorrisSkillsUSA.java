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

/*
 * This OpMode executes a Tank Drive control TeleOp a direct drive robot
 * The code is structured as an Iterative OpMode
 *
 * In this mode, the left and right joysticks control the left and right motors respectively.
 * Pushing a joystick forward will make the attached motor drive forward.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Morris Tank Teleop", group="Robot")
//@Disabled
public class MorrisSkillsUSA extends OpMode{

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
    double clawOffset = 0;
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation

    // --- Constants for Drivetrain ---
    public static final double MOTOR_ENCODER_TICKS_PER_REVOLUTION = 537.7; // For goBILDA 312 RPM
    public static final double MOTOR_MAX_RPM = 312.0;
    public static final double MOTOR_MAX_RPS = MOTOR_MAX_RPM / 60.0;
    public static final double MAX_MOTOR_TICKS_PER_SECOND = MOTOR_MAX_RPS * MOTOR_ENCODER_TICKS_PER_REVOLUTION; // Approx 2796.04

    public static final double TRACK_PITCH_MM = 24.0;
    public static final double TRACK_PITCH_INCHES = TRACK_PITCH_MM / 25.4; // Approx 0.94488
    public static final double TRACK_SPROCKET_TEETH = 12.0;
    public static final double MOTOR_TO_DRIVEN_GEAR_RATIO = 36.0 / 24.0; // 1.5
    public static final double OVERALL_GEAR_RATIO_MOTOR_TO_SPROCKET = MOTOR_TO_DRIVEN_GEAR_RATIO; // Assuming 1:1 timing pulleys

    public static final double SPROCKET_CIRCUMFERENCE_INCHES = TRACK_SPROCKET_TEETH * TRACK_PITCH_INCHES; // Approx 11.33856
    public static final double MOTOR_TICKS_PER_SPROCKET_REV = MOTOR_ENCODER_TICKS_PER_REVOLUTION * OVERALL_GEAR_RATIO_MOTOR_TO_SPROCKET; // Approx 806.55

    public static final double TICKS_PER_INCH = MOTOR_TICKS_PER_SPROCKET_REV / SPROCKET_CIRCUMFERENCE_INCHES; // Approx 71.1329

    // YOUR CALCULATED TRACK WIDTH - VERIFY THIS MEASUREMENT ON YOUR ROBOT
    public static final double TRACK_WIDTH_INCHES = 12.9931; // From (17 + 7/8) - (124 / 25.4)

    public static final double TICKS_PER_DEGREE_ONE_TRACK = calculateTicksPerDegree(); // Approx 4.0326

    // Helper to calculate Ticks Per Degree
    private static double calculateTicksPerDegree() {
        double turningCircleCircumference = Math.PI * TRACK_WIDTH_INCHES;
        double distancePerTrackFor360Turn = turningCircleCircumference / 2.0;
        double motorTicksFor360TurnOneTrack = distancePerTrackFor360Turn * TICKS_PER_INCH;
        return motorTicksFor360TurnOneTrack / 360.0;
    }

    private static final boolean USE_WEBCAM = true;
    public static final double MID_SERVO   =  0 ;
    public static final double CLAW_SPEED  = 0.02 ;        // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.50 ;   // Run arm motor up at 50% power
    public static final double ARM_DOWN_POWER  = -0.25 ;   // Run arm motor down at -25% power
    public static final int ARM_INCREMENT = 75;
    public static int armTargetPos;
    public static final int ARM_MIN = 0;
    public static final int SLIDE_INCREMENT = 100;
    public static final int SLIDE_MAX = 1000;
    public static final int SLIDE_MIN = 0;
    public static double clawPosition;
    public static final double CLAW_STOWED = 0.55;
    public static double wristPosition;
    public static final double WRIST_FOLDED = 0.28;
    public static boolean turtleMode = false;
    public static boolean isBackButtonPressed = false;
    public static final double TURTLE_MODE_SPEED = .2;
    public static double driveSpeed = 1;

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
        visionPortal = builder.build();
        // Define and Initialize Motors
        left_front_drive = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        left_rear_drive = hardwareMap.get(DcMotorEx.class, "left_rear_drive");
        right_front_drive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        right_rear_drive = hardwareMap.get(DcMotorEx.class, "right_rear_drive");
        arm_motor = hardwareMap.get(DcMotorEx.class, "lift");
        slide_motor = hardwareMap.get(DcMotorEx.class, "slide");

        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//         Define and initialize servos.
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "grabber");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        left_front_drive.setDirection(DcMotor.Direction.FORWARD);
        left_rear_drive.setDirection(DcMotor.Direction.FORWARD);
        right_front_drive.setDirection(DcMotor.Direction.REVERSE);
        right_rear_drive.setDirection(DcMotor.Direction.REVERSE);
        arm_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        slide_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_rear_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_rear_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set target positions to whatever position they are at on robot startup so that the arm motor and servos don't move on initialization or play
        armTargetPos = arm_motor.getCurrentPosition();
        wristPosition = WRIST_FOLDED;
        clawPosition = CLAW_STOWED;

//        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
//
//         arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//         slide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        double driveInput;
        double turnInput;
        double leftPowerRaw;
        double rightPowerRaw;
        double denominator;


        //Turtle mode slows driveInput motors when toggled with back button
        // This uses a method called button latching which only toggles when you release the button
        // to prevent accidentally reading multiple button presses
        if(gamepad1.back && !isBackButtonPressed) {
            isBackButtonPressed = true;
        } else if (!gamepad1.back && isBackButtonPressed) {
            isBackButtonPressed = false;
            turtleMode = !turtleMode;
        }

        if (turtleMode)
        {
            driveSpeed = TURTLE_MODE_SPEED;
        } else driveSpeed = 1;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
        driveInput = -gamepad1.left_stick_y;
        turnInput = -gamepad1.left_stick_x;
        // --- Calculate Raw Power Values for Each Side (Arcade Logic) ---
        leftPowerRaw = driveInput - turnInput;
        rightPowerRaw = driveInput + turnInput;

        // --- Normalize Power Values (Important!) ---
        // This ensures that if the combined inputs exceed 1.0 (e.g., full forward + full turn),
        // the proportions are maintained, and the maximum output for one motor doesn't exceed 1.0.
        denominator = Math.max(Math.abs(driveInput) + Math.abs(turnInput), 1.0);
        double leftScaledPower = leftPowerRaw / denominator;
        double rightScaledPower = rightPowerRaw / denominator;

        // --- Apply Overall Drive Speed Multiplier (e.g., for turtle mode) ---
        // Assuming 'driveSpeed' is a variable like 1.0 for full speed, 0.5 for turtle mode
        double finalLeftPower = leftScaledPower * driveSpeed;
        double finalRightPower = rightScaledPower * driveSpeed;

        // --- Convert Scaled Power to Target Velocity (Ticks Per Second) ---
        double leftTargetVelocity = finalLeftPower * MAX_MOTOR_TICKS_PER_SECOND;
        double rightTargetVelocity = finalRightPower * MAX_MOTOR_TICKS_PER_SECOND;

        // --- Set Motor Velocities ---
        // Ensure motors are in RUN_USING_ENCODER mode (should be set in init() and after any RUN_TO_POSITION)
        left_front_drive.setVelocity(leftTargetVelocity);
        left_rear_drive.setVelocity(leftTargetVelocity);
        right_front_drive.setVelocity(rightTargetVelocity);
        right_rear_drive.setVelocity(rightTargetVelocity);

        // Claw code
        clawPosition += (gamepad1.right_trigger - gamepad1.left_trigger) * .03;
        claw.setPosition(clawPosition);
        //Claw limit
        if (clawPosition > 1)
            clawPosition = 1;
        else if (clawPosition < 0) {
            clawPosition = 0;
        }

        //Wrist code
        if (gamepad1.right_bumper) {
            wristPosition += 0.01;
        }
        else if (gamepad1.left_bumper) {
            wristPosition -= 0.01;
        }
       //Wrist limit
        if (wristPosition > 1)
            wristPosition = 1;
        else if (wristPosition < 0) {
            wristPosition = 0;
        }
        wrist.setPosition(wristPosition);

        // Use gamepad buttons to move the arm up (Y) and down (A)
        armTargetPos = armTargetPos - (int) (gamepad1.right_stick_y * ARM_INCREMENT);
        arm_motor.setTargetPosition(armTargetPos);
        slide_motor.setTargetPosition(slide_motor.getCurrentPosition() + (int) (gamepad1.right_stick_x * SLIDE_INCREMENT));
        arm_motor.setPower(1);
        arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide_motor.setPower(1);
        slide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        if (arm_motor.getCurrentPosition() > ARM_MAX)
//            arm_motor.setTargetPosition((int) ARM_MAX);
//        else if (arm_motor.getCurrentPosition() < ARM_MIN)
//            arm_motor.setTargetPosition((int) ARM_MIN);
//
//        if (slide_motor.getCurrentPosition() > SLIDE_MAX)
//            slide_motor.setTargetPosition((int) SLIDE_MAX);
//        else if (slide_motor.getCurrentPosition() < SLIDE_MIN)
//            slide_motor.setTargetPosition((int) SLIDE_MIN);

        // Send telemetry message to signify robot running;
        telemetry.addData("Drive Input", "%.2f", driveInput);
        telemetry.addData("Turn Input", "%.2f", turnInput);
        telemetry.addData("Left Scaled", "%.2f", leftScaledPower);
        telemetry.addData("Right Scaled", "%.2f", rightScaledPower);
        telemetry.addData("Drive Speed Mod", "%.2f", driveSpeed);
        telemetry.addData("Left Target TPS", "%.2f", leftTargetVelocity);
        telemetry.addData("Right Target TPS", "%.2f", rightTargetVelocity);
        telemetry.addData("claw position: ", clawPosition);
        telemetry.addData("wrist position: ", wristPosition);
        telemetry.addData("arm position: ", arm_motor.getCurrentPosition());
        telemetry.addData("slide position: ", slide_motor.getCurrentPosition());
        telemetry.addData("driveInput",  "%.2f", driveInput);
        telemetry.addData("right", "%.2f", turnInput);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}

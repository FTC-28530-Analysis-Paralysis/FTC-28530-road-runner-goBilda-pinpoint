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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name="Robot: Teleop Tank", group="Robot")
//@Disabled
public class RobotTeleopTank_Iterative extends OpMode{

    /* Declare OpMode members. */
    public DcMotor left_front_drive = null;
    public DcMotor left_rear_drive = null;
    public DcMotor right_front_drive = null;
    public DcMotor right_rear_drive = null;
    public DcMotor lift_motor = null;
    public DcMotor slide_motor = null;
    public Servo wrist = null;
    public Servo   claw   = null;

    double clawOffset = 0;

    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;        // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.50 ;   // Run arm motor up at 50% power
    public static final double ARM_DOWN_POWER  = -0.25 ;   // Run arm motor down at -25% power

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        left_front_drive = hardwareMap.get(DcMotor.class, "left_front_drive");
        left_rear_drive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");
        right_rear_drive = hardwareMap.get(DcMotor.class, "right_rear_drive");
//        lift_motor = hardwareMap.get(DcMotor.class, "lift");
//        slide_motor = hardwareMap.get(DcMotor.class, "slide");

//         Define and initialize servos.
//        wrist = hardwareMap.get(Servo.class, "wrist");
//        claw = hardwareMap.get(Servo.class, "grabber");
//        wrist.setPosition(MID_SERVO);
//        claw.setPosition(MID_SERVO);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left_rear_drive.setDirection(DcMotor.Direction.REVERSE);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        right_rear_drive.setDirection(DcMotor.Direction.FORWARD);

        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_rear_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_rear_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



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
        double drive;
        double rot;
        double denominator;


        // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
        drive = -gamepad1.left_stick_y;
        rot = gamepad1.left_stick_x;
        denominator = Math.max(drive + rot,1);
        left_front_drive.setPower((drive - rot)/denominator);
        left_rear_drive.setPower((drive - rot)/denominator);
        right_front_drive.setPower((drive + rot)/denominator);
        right_rear_drive.setPower((drive + rot)/denominator);

        // Use gamepad drive & right Bumpers to open and close the claw
        if (gamepad1.right_bumper)
            clawOffset += CLAW_SPEED;
        else if (gamepad1.left_bumper)
            clawOffset -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
//        leftClaw.setPosition(MID_SERVO + clawOffset);
//        rightClaw.setPosition(MID_SERVO - clawOffset);

        // Use gamepad buttons to move the arm up (Y) and down (A)
//        if (gamepad1.y)
//            leftArm.setPower(ARM_UP_POWER);
//        else if (gamepad1.a)
//            leftArm.setPower(ARM_DOWN_POWER);
//        else
//            leftArm.setPower(0.0);

        // Send telemetry message to signify robot running;
        telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        telemetry.addData("drive",  "%.2f", drive);
        telemetry.addData("right", "%.2f", rot);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}

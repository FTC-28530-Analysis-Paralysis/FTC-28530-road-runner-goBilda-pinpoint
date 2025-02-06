/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.


 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * This is (mostly) the OpMode used in the goBILDA Robot in 3 Days for the 24-25 Into The Deep FTC Season.
 * https://youtube.com/playlist?list=PLpytbFEB5mLcWxf6rOHqbmYjDi9BbK00p&si=NyQLwyIkcZvZEirP (playlist of videos)
 * I've gone through and added comments for clarity. But most of the code remains the same.
 * This is very much based on the code for the Starter Kit Robot for the 24-25 season. Those resources can be found here:
 * https://www.gobilda.com/ftc-starter-bot-resource-guide-into-the-deep/
 *
 * There are three main additions to the starter kit bot code, mecanum drive, a linear slide for reaching
 * into the submersible, and a linear slide to hang (which we didn't end up using)
 *
 * the drive system is all 5203-2402-0019 (312 RPM Yellow Jacket Motors) and it is based on a Strafer chassis
 * The arm shoulder takes the design from the starter kit robot. So it uses the same 117rpm motor with an
 * external 5:1 reduction
 *
 * The drivetrain is set up as "field centric" with the internal control hub IMU. This means
 * when you push the stick forward, regardless of robot orientation, the robot drives away from you.
 * We "took inspiration" (copy-pasted) the drive code from this GM0 page
 * (PS GM0 is a world class resource, if you've got 5 mins and nothing to do, read some GM0!)
 * https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#field-centric
 */

/**
 * TODO: Write code for the following ideas, then test:
 *       1) Test Pre-hang routine (gamepad1.y) - adjust the ARM_ATTACH_HANGING_HOOK angle as needed
 *       2) Test Hang routine (gamepad1.x) - can disable if it doesn't seem good
 *       3) Fix horizontal limit code so that slide extends more as it goes up - Mr. Morris needs to see this and maybe add some telemetry to troubleshoot
 *       4) Try variants of code for arm and slide that take cycleTime into account for smoother operation
 *
 * DONE: 1) Create 42" horizontal limit to keep robot arm and slide operating in the legal envelope
 *       2) Implement finite state machine model to track arm preset movements, with delays for arm rotation and slide extension to manage whiplash
 *       3) Intake successful routine (gamepad1.b) - rotate arm up, retract slide
 *       4) Pre-hang routine (gamepad1.y) - fold wrist, retract slide, rotate to hang hook position
 *       5) Hang routine? (gamepad1.x) - rotate arm and drive forward a little bit
 */

@TeleOp(name="Competition-Teleop-Modified", group="0Competition")
//@Disabled
public class CompetitionTeleopModified extends LinearOpMode {



    /* Declare OpMode members. */
    public DcMotor  leftFrontDrive   = null; //the left drivetrain motor
    public DcMotor  rightFrontDrive  = null; //the right drivetrain motor
    public DcMotor  leftBackDrive    = null;
    public DcMotor  rightBackDrive   = null;
    public DcMotor  armMotor         = null; //the arm motor
    public DcMotor  slideMotor = null;
    public CRServo  intake           = null; //the active intake servo
    public Servo    wrist            = null; //the wrist servo
    private int sequenceState = 0;
    private ElapsedTime sequenceTimer = new ElapsedTime();

    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
    To find this, we first need to consider the total gear reduction powering our arm.
    First, we have an external 20t:100t (5:1) reduction created by two spur gears.
    But we also have an internal gear reduction in our motor.
    The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
    reduction of ~50.9:1. (more precisely it is 250047/4913:1)
    We can multiply these two ratios together to get our final reduction of ~254.47:1.
    The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
    counts per rotation of the arm. We divide that by 360 to get the counts per degree. */

    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation

    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_HIGH is set to
    99 * ARM_TICKS_PER_DEGREE. This asks the arm to move 99° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 5   * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 15  * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 90  * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_HIGH  = 99  * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 106 * ARM_TICKS_PER_DEGREE;
    final double ARM_HANG                  = 10  * ARM_TICKS_PER_DEGREE;

    final static double ARM_RETRACTED_LENGTH_MM = 495;
    final static double MAX_FORWARD_EXTENSION_MM = (42 - 5.5) * 25.4; // 42 inches is max horizontal envelope robot can extend in,
                                                                // 5.5 inches is how far back we need to reach for the ascent

    // These are the arm states that the gamepad1 dpad buttons will toggle through
    private enum ArmState{HIGH_BASKET, COLLECT, FOLDED, CLEAR_BARRIER, INTAKE_COMPLETE, PRE_HANG, HANG, IDLE}
    private ArmState currentArmState = ArmState.FOLDED;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT    =  1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    = -0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN   = 0.1667;
    final double WRIST_FOLDED_OUT  = 0.5;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;

    final double SLIDE_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    final double SLIDE_COLLAPSED = 0 * SLIDE_TICKS_PER_MM;
    final double SLIDE_SCORING_IN_LOW_BASKET = 0 * SLIDE_TICKS_PER_MM;
    final double SLIDE_SCORING_IN_HIGH_BASKET = 420 * SLIDE_TICKS_PER_MM;

    double slideTargetPosition = SLIDE_COLLAPSED;

    final double BRAKE_GAIN = .5; // Limit braking effect to 50% of max speed
    double brake = 0;

    double cycleTime = 0;
    double loopTime = 0;
    double oldTime = 0;

    double armSlideComp = 0;

    @Override
    public void runOpMode() {
        /// Mr. Morris: Here is code for the time tracker. This gets reset once the driver presses
        ///             play and then is used to set other time tracking variables.
        ElapsedTime runtime = new ElapsedTime();

       /*
       These variables are private to the OpMode, and are used to control the drivetrain.
        */
        double left;
        double right;
        double forward;
        double rotate;
        double max;

        /* Define and Initialize Motors */
        leftFrontDrive  = hardwareMap.dcMotor.get("left_front_drive");
        leftBackDrive   = hardwareMap.dcMotor.get("left_rear_drive");
        rightFrontDrive = hardwareMap.dcMotor.get("right_front_drive");
        rightBackDrive  = hardwareMap.dcMotor.get("right_rear_drive");
        armMotor        = hardwareMap.dcMotor.get("arm"); //the arm motor
        slideMotor      = hardwareMap.dcMotor.get("slide");

        /* we need to reverse the left side of the drivetrain so it doesn't turn when we ask all the
        drive motors to go forward. */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        /* Setting zeroPowerBehavior to BRAKE enables a "brake4 mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5,CurrentUnit.AMPS);

        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Define and initialize servos.*/
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist  = hardwareMap.get(Servo.class, "wrist");

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_OUT);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        /* Wait for the game driver to press play */
        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive()) {
            // This calls the method at the bottom of the file which has a switch statement to move
            // the arm, slide, and wrist together in coordinated movements.
            updateArmState(currentArmState);

            // Hold left trigger to apply a variable slowing effect to all wheel and arm movements - all the way pressed results in 50% speed reduction
            brake = (1 - gamepad1.left_trigger * BRAKE_GAIN); //TODO: Currently this even affects moving to presets - test and remove that if it's not needed

            double y = -squareInputWithSign(gamepad1.left_stick_y) * brake; // left stick is negative when up so flip that with negative sign
            double x = squareInputWithSign(gamepad1.left_stick_x) * brake;
            double rx = squareInputWithSign(gamepad1.right_stick_x) * brake;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);

            /* TECH TIP: If Else statement:
            We're using an else if statement on "gamepad1.x" and "gamepad1.b" just in case
            multiple buttons are pressed at the same time. If the driver presses both "a" and "x"
            at the same time. "a" will win over and the intake will turn on. If we just had
            three if statements, then it will set the intake servo's power to multiple speeds in
            one cycle. Which can cause strange behavior. */

            if (gamepad1.left_bumper) {
                intake.setPower(INTAKE_COLLECT);
            }
            else if (gamepad1.right_bumper) {
                intake.setPower(INTAKE_DEPOSIT);
            }
            else if (gamepad2.left_bumper) {
                intake.setPower(INTAKE_COLLECT);
            }
            else if (gamepad2.right_bumper) {
                intake.setPower(INTAKE_DEPOSIT);
            }
            else {
                intake.setPower(INTAKE_OFF);
            }



            // TODO: Make armPosition movement dependent on cycleTime like the slide example below - should smooth arm movement
            armPosition = armPosition - gamepad2.left_stick_y * 30 * brake; //Arm speed

            /* Here we implement a set of if else statements to set our arm to different scoring positions.
            We check to see if a specific button is pressed, and then move the arm (and sometimes
            intake and wrist) to match. For example, if we click the right bumper we want the robot
            to start collecting. So it moves the armPosition to the ARM_COLLECT position,
            it folds out the wrist to make sure it is in the correct orientation to intake, and it
            turns the intake on to the COLLECT mode.*/

            if (gamepad1.dpad_left) {
                /* This raises the arm up just enough to clear the barrier to collect samples in the submersible */
                currentArmState = ArmState.CLEAR_BARRIER;
            }

            else if (gamepad1.dpad_down){
                /* This lowers the arm to the ground */
                sequenceTimer.reset();
                currentArmState = ArmState.COLLECT;
            }

            else if (gamepad1.dpad_up){
                /* This raises and extends the arm up to score a sample in the high basket */
                sequenceTimer.reset();
                currentArmState = ArmState.HIGH_BASKET;
            }

            else if (gamepad1.dpad_right){
                /* this moves the arm down, brings the slide in, and folds the wrist in */
                updateArmState(ArmState.FOLDED);
            }

            else if (gamepad1.b){
                /* this rotates the arm up, retracts the slide, and turns off the intake servo */
                updateArmState(ArmState.INTAKE_COMPLETE);
            }

            else if (gamepad1.y){
                /* this rotates the arm up, retracts the slide, and turns off the intake servo */
                updateArmState(ArmState.PRE_HANG);
            }

            else if (gamepad1.x){
                /* this rotates the arm down and drives forward a bit to engage the hook and then draw the robot up */
                updateArmState(ArmState.HANG);
            }


           /* This is probably my favorite piece of code on this robot. It's a clever little software
           solution to a problem the robot has.
           This robot has an extending lift on the end of an arm shoulder. That arm shoulder should
           run to a specific angle, and stop there to collect from the field. And the angle that
           the shoulder should stop at changes based on how long the arm is (how far the lift is extended)
           so here, we add a compensation factor based on how far the lift is extended.
           That comp factor is multiplied by the number of mm the lift is extended, which
           results in the number of degrees we need to fudge our arm up by to keep the end of the arm
           the same distance from the field.
           Now we don't need this to happen when the arm is up and in scoring position. So if the arm
           is above 45°, then we just set armSlideComp to 0. It's only if it's below 45° that we set it
           to a value. */
            if (armPosition < 45 * ARM_TICKS_PER_DEGREE){
                armSlideComp = (0.25568 * slideTargetPosition);
            }
            else{
                armSlideComp = 0;
            }

            /* Here we set the target position of our arm to match the variable that was selected
            by the driver. We add the armPosition Variable to our armPositionFudgeFactor, before adding
            our armSlideComp, which adjusts the arm height for different lift extensions.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/
            if (armPosition/ARM_TICKS_PER_DEGREE > 138)
            {
                armPosition = 138 * ARM_TICKS_PER_DEGREE;
            }
            armMotor.setTargetPosition((int) (armPosition + armSlideComp));

            ((DcMotorEx) armMotor).setVelocity(2100 * brake); //TODO: The brake is being applied to preset velocity, remove if undesired
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* Here we set the lift position based on the driver input.
            This is a.... weird, way to set the position of a "closed loop" device. The lift is run
            with encoders. So it knows exactly where it is, and there's a limit to how far in and
            out it should run. Normally with mechanisms like this we just tell it to run to an exact
            position. This works a lot like our arm. Where we click a button and it goes to a position, then stops.
            But the drivers wanted more "open loop" controls. So we want the lift to keep extending for
            as long as we hold the bumpers, and when we let go of the bumper, stop where it is at.
            This allows the driver to manually set the position, and not have to have a bunch of different
            options for how far out it goes. But it also lets us enforce the end stops for the slide
            in software. So that the motor can't run past its endstops and stall.
            We have our SlidePosition variable, which we increment or decrement for every cycle (every
            time our main robot code runs) that we're holding the button. Now since every cycle can take
            a different amount of time to complete, and we want the lift to move at a constant speed,
            we measure how long each cycle takes with the cycleTime variable. Then multiply the
            speed we want the lift to run at (in mm/sec) by the cycleTime variable. There's no way
            that our lift can move 2800mm in one cycle, but since each cycle is only a fraction of a second,
            we are only incrementing it a small amount each cycle. */

            /* TODO: Mr. Morris: The first time I read their code I didn't get the elegance of their solution.
             *                   We might want to try something similar to their original rather than what we
             *                   have now. But it will need testing. This would be how I would rewrite it for
             *                   analog (stick) input:
             *  slideTargetPosition += gamepad2.right_stick_x * 2800 * cycleTime;
             */

            slideTargetPosition += gamepad2.right_stick_x * 25 * brake; //Slide speed

            /* here we check to see if the lift is trying to go higher than the maximum extension.
            if it is, we set the variable to the max. */
            if (slideTargetPosition > SLIDE_SCORING_IN_HIGH_BASKET){
                slideTargetPosition = SLIDE_SCORING_IN_HIGH_BASKET;
            }
            //same as above, we see if the lift is trying to go below 0, and if it is, we set it to 0.
            if (slideTargetPosition < 0){
                slideTargetPosition = 0;
            }

            // If desired slideTargetPosition is outside the 42" horizontal extension, limit it.
            // Convert from mm to encoder ticks
            slideMotor.setTargetPosition((int) (Math.min(slideTargetPosition, maxSlideLengthForGivenAngle() * SLIDE_TICKS_PER_MM)));

            ((DcMotorEx) slideMotor).setVelocity(2100 * brake); //TODO: The brake is being applied to preset velocity, remove if undesired
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (((DcMotorEx) armMotor).isOverCurrent()){
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }

            /* This is how we check our loop time. We create three variables:
                loopTime is the current time when we hit this part of the code.
                cycleTime is the amount of time in seconds our current loop took.
                oldTime is the time in seconds that the previous loop started at.
            We find cycleTime by just subtracting the old time from the current time. For example,
            lets say it is 12:01.1, and then a loop goes by and it's 12:01.2. We can take the current
            time (12:01.2) and subtract the oldTime (12:01.1) and we're left with just the difference,
            0.1 seconds. */
            loopTime = getRuntime();
            cycleTime = loopTime - oldTime;
            oldTime = loopTime;

            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("arm Target Position: ", armMotor.getTargetPosition());
            telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
            telemetry.addData("arm angle: ", armMotor.getCurrentPosition()/ARM_TICKS_PER_DEGREE);
            telemetry.addData("normalized arm angle: ", getNormalizedArmAngle());
            telemetry.addData("Current arm state: " , currentArmState);
            telemetry.addData("slide variable", slideTargetPosition);
            telemetry.addData("slide Target Position",slideMotor.getTargetPosition());
            telemetry.addData("slide current position", slideMotor.getCurrentPosition());
            telemetry.addData("slide current mm", slideMotor.getCurrentPosition()/SLIDE_TICKS_PER_MM);
            telemetry.addData("Calculated horizontal extension", Math.cos(Math.toRadians(armMotor.getCurrentPosition()/ARM_TICKS_PER_DEGREE -33.3))*(ARM_RETRACTED_LENGTH_MM+(slideMotor.getCurrentPosition()/SLIDE_TICKS_PER_MM)));
            telemetry.addData("slideMotor Current:",((DcMotorEx) slideMotor).getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }

    /*  Mr. Morris - I got this idea from https://github.com/alan412/LearnJavaForFTC/blob/master/LearnJavaForFTC.pdf
     *  This function squares input but keeps the sign. This is used to alter joystick input to be less sensitive at small values.
     *  This works because small values less than one become even smaller when squared but values near one have little change. In
     *  this way, it is easier to have fine control over the robot to make small movements.
     */
    private double squareInputWithSign(double input) {
        if (input < 0) {
            return -input * input;
        } else {
            return input * input;
        }
    }

    /**
     * Mr. Morris:
     * This method is a finite state machine implementation to track the possible positions of the arm, slide, and wrist. The
     * main reason this was implemented is because Mahlon asked for a time delay when moving from the HIGH_BASKET down. I
     * interpreted this as hitting the button to go to the COLLECT, however it could be adjusted to behave how the drivers want.
     * Currently it has a timer in the COLLECT case so that the arm delays half a second while the slide retracts. There are
     * certainly simpler solutions to this problem, but this one is more robust and gives us the framework to make all sorts of
     * fancy transitions between states while still being readable. The magic happens at the top of the while(opModeIsActive())
     * loop. It calls this method to update the arm state every loop. Later in the program buttons will change the currentState
     * and the next program loop when the method is called it will update the positions. By being called every program loop
     * regardless of button press it can track timers and even flow from one arm state to another if we want it to. Currently I
     * have each of the states moving to the IDLE state after finishing so that gamepad2 can change the arm and slide positions.
     * Without the IDLE state, updateArmState constantly overrides the gamepad2 controller input.
     * @param state This is an enum of ArmState type. Enums are a way of creating a sort of list of options that can be chosen
     *              by name in a switch statement like this. This makes code more readable and less prone to errors. It is a
     *              description of a position the arm, slide, and wrist can be in (e.g. FOLDED, COLLECT, etc.)
     */
    private void updateArmState(ArmState state){
        switch (state){
            case HIGH_BASKET:
                // We have a delay where the arm rotates first, then slides out to prevent the robot from tipping
                armPosition = ARM_SCORE_SAMPLE_IN_HIGH;
                if (sequenceTimer.seconds() > 0.7) { // Adjust this time as needed to prevent robot tipping
                    slideTargetPosition = SLIDE_SCORING_IN_HIGH_BASKET;
                    currentArmState = ArmState.IDLE;
                }
                break;
            case COLLECT:
                // We have a delay where the arm slides in first, then rotates to prevent collision with the high basket
                slideTargetPosition = SLIDE_COLLAPSED;
                if (sequenceTimer.seconds() > 0.5){ // Adjust this time as needed to prevent collision
                    armPosition = ARM_COLLECT;
                    currentArmState = ArmState.IDLE;
                }
                break;
            case FOLDED:
                slideTargetPosition = SLIDE_COLLAPSED;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
                currentArmState = ArmState.IDLE;
                break;
            case CLEAR_BARRIER:
                armPosition = ARM_CLEAR_BARRIER;
                currentArmState = ArmState.IDLE;
                break;
            case INTAKE_COMPLETE:
                intake.setPower(INTAKE_OFF);
                armPosition = ARM_CLEAR_BARRIER;
                slideTargetPosition = SLIDE_COLLAPSED;
                currentArmState = ArmState.IDLE;
                break;
            case PRE_HANG:
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
                armPosition = ARM_ATTACH_HANGING_HOOK;
                slideTargetPosition = SLIDE_COLLAPSED;
                currentArmState = ArmState.IDLE;
                break;
            case HANG:
                armPosition = ARM_HANG;
                leftFrontDrive.setPower(.5);
                leftBackDrive.setPower(.5);
                rightFrontDrive.setPower(.5);
                rightBackDrive.setPower(.5);
                if (sequenceTimer.seconds() > 2) {
                    leftFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    rightBackDrive.setPower(0);
                    currentArmState = ArmState.IDLE;
                }
                break;
            case IDLE:
                break;
            default:
                telemetry.addData("Error", "Invalid arm state: " + state);
                telemetry.update();
                throw new IllegalArgumentException("Invalid arm state: " + state);
        }
    }

    // TODO: TEST!
    private double maxSlideLengthForGivenAngle(){

        double hypotenuse = MAX_FORWARD_EXTENSION_MM / Math.cos(Math.toRadians(getNormalizedArmAngle())); // Using trigonometry to calculate total length: cos(θ) = adjacent / hypotenuse
        return hypotenuse - ARM_RETRACTED_LENGTH_MM; // Total arm length minus retracted length leaves the slide extension amount
    }

    private double getNormalizedArmAngle(){
        return armMotor.getCurrentPosition()/ARM_TICKS_PER_DEGREE - 33.3;
    }
}
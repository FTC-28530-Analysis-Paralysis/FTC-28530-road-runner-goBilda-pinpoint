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

package org.firstinspires.ftc.teamcode.competition;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.PinpointDrive;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="ConfigurableAutonomousRR", group="0Competition", preselectTeleOp = "CompetitionTeleopModified") // I added preselectTeleop so you don't have to select the next one manually in match
//@Disabled
public class ConfigurableAutonomousRR extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    boolean isLeftStart = true;
    double  startingPause = 0;
    private PinpointDrive drive;
    private Pose2d initialPose;
    final double WRIST_FOLDED_OUT  = 0.2;
    Servo wrist = null;

    private DcMotor arm = null;
    @Override
    public void runOpMode() {
        // instantiate the PinpointDrive at a particular pose.

        // Select Alliance, starting point, and route options
        configAutonomous();
        //sets that arm to be a little up above the ground when started

        arm = hardwareMap.get(DcMotor.class, "arm"); //the arm motor
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);// allows the arm move
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // this makes the position the arm is before its moved the "starting" position
        arm.setTargetPosition(10); // this is the position above the ground
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // makes the arm be able to move
        arm.setPower(.1); // gives the arm power

        wrist = hardwareMap.get(Servo.class, "wrist");

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        wrist.setPosition(WRIST_FOLDED_OUT);
        if(isLeftStart){
            // instantiate the PinpointDrive at the center of the second-from-left tile
            initialPose = new Pose2d(37, 61.7, Math.toRadians(-90));
            drive = new PinpointDrive(hardwareMap, initialPose);
            Actions.runBlocking(drive.actionBuilder(initialPose)
                    .setTangent(Math.toRadians(0))
                    .lineToX(49)
                    .splineToLinearHeading(new Pose2d(37,35,Math.toRadians(-90)), Math.toRadians(-90))
                    .lineToY(17)
                    .splineToLinearHeading(new Pose2d(40,12,Math.toRadians(-110)), Math.toRadians(75))
                    .setTangent(Math.toRadians(75))
                    .lineToY(58)
                    .lineToY(12)
                    .splineToLinearHeading(new Pose2d(53,12,Math.toRadians(-90)), Math.toRadians(85))
                    .setTangent(Math.toRadians(85))
                    .lineToY(55)
                    .lineToY(12)
                    .splineToLinearHeading(new Pose2d(61,10,Math.toRadians(-90)), Math.toRadians(90))
                    .lineToY(46)
                    .splineToLinearHeading(new Pose2d(25, 12, Math.toRadians(180)), Math.toRadians(160))
                    .build());
        } else {
            // instantiate the PinpointDrive at the center of the third-from-right tile
            initialPose = new Pose2d(-11.8, 61.7, Math.toRadians(-90));
            drive = new PinpointDrive(hardwareMap, initialPose);
            Actions.runBlocking(drive.actionBuilder(initialPose)
                    .setTangent(Math.toRadians(0))
                    .lineToX(-49)
                    .splineToLinearHeading(new Pose2d(-35,32,Math.toRadians(-90)), Math.toRadians(-90))
                    .lineToY(20)
                    .splineToLinearHeading(new Pose2d(-45,12,Math.toRadians(-90)), Math.toRadians(90))
                    .lineToY(53)
                    .lineToY(12)
                    .splineToLinearHeading(new Pose2d(-53,10,Math.toRadians(-90)), Math.toRadians(90))
                    .lineToY(53)
                    .lineToY(12)
                    .splineToLinearHeading(new Pose2d(-61,10,Math.toRadians(-90)), Math.toRadians(90))
                    .lineToY(55)
                    .build());
        }

    }
    /**
     * This method used a technique called latching with all the button presses where it only toggles
     * when you let go of the button. This keeps from accidentally registering multiple button presses
     * when you hold the button for too long.
     */
    private void configAutonomous(){
        int selection = 0;
        final int SELECTION_COUNT = 4; // Number of options in selection list.
        boolean dpadDownPressed = false, dpadUpPressed = false, dpadRightOrLeftPressed = false, dpadRightPressed = false, dpadLeftPressed = false;
        boolean configComplete = false;

        // This loops until the x
        while(!isStarted() && !configComplete){
            // This is the first example of latching. This while loop can cycle hundreds of times a
            // second. This waits until the dpad_down button is not pressed before it increments
            // the selection
            if(gamepad1.dpad_down) dpadDownPressed = true;
            else if(dpadDownPressed && !gamepad1.dpad_down){
                dpadDownPressed = false;
                selection += 1;
                selection = selection % SELECTION_COUNT; // cycles around to beginning of list after the end
                // % means the remainder after dividing
            }

            if(gamepad1.dpad_up) dpadUpPressed = true;
            else if(dpadUpPressed && !gamepad1.dpad_up){
                dpadUpPressed = false;
                if (selection >0) selection -= 1;
            }

            // The following blocks display an arrow next to the option currently being selected and wait for
            // you to toggle that option by pressing either dpad_left or dpad_right
            telemetry.addLine("Press Dpad Up/Down to choose an option, and Left/Right to change options");
            if(selection == 1) {
                telemetry.addData("-->Starting Position: ", isLeftStart ?"Left": "Right");
                if(gamepad1.dpad_right || gamepad1.dpad_left) dpadRightOrLeftPressed = true;
                else if(dpadRightOrLeftPressed && !(gamepad1.dpad_right || gamepad1.dpad_left)){
                    dpadRightOrLeftPressed = false;
                    isLeftStart = !isLeftStart;
                }
            } else telemetry.addData("Starting Position: ", isLeftStart ?"Left": "Right");

            if(selection == 2) {
                telemetry.addData("-->Starting Pause: ", startingPause +" seconds");
                if(gamepad1.dpad_right) dpadRightPressed = true;
                else if(dpadRightPressed && !gamepad1.dpad_right){
                    dpadRightPressed = false;
                    startingPause += 0.5;
                }
                if(gamepad1.dpad_left) dpadLeftPressed = true;
                else if(dpadLeftPressed && !gamepad1.dpad_left){
                    dpadLeftPressed = false;
                    startingPause -= 0.5;
                }
            } else telemetry.addData("Starting Pause: ", startingPause +" seconds");

            telemetry.addLine("\nPress [x] to complete");

            // Press x to end Autonomous Configuration
            if(gamepad1.x) configComplete = true;
            telemetry.update();
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Autonomous Configuration Complete.  Press Play to start OpMode.");
        telemetry.update();
    }
}

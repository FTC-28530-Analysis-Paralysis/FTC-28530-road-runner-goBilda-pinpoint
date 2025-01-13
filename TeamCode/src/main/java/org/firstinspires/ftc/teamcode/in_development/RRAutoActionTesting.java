package org.firstinspires.ftc.teamcode.in_development;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunnerFiles.MecanumDrive;


// This program follows the tutorial video from FTC 6282 Simi Valley Robotics: https://youtu.be/uBwVSRxvpB8?si=0o4t_w6YdL_58o8R
// It is an example of how to add actions for other mechanisms (a servo here) to complete during a roadrunner autonomous routine.
@Autonomous(name="RR Auto Action Testing")
public class RRAutoActionTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Servo intakeLeft = hardwareMap.servo.get("servo");

        waitForStart();

        // runBlocking runs actions in blocking sequence so each action waits until the previous one is done
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,0))
                .lineToX(64)
                .stopAndAdd(new PatientServoAction(intakeLeft, 1)) //
                .lineToX(0)
                .stopAndAdd(new ServoAction(intakeLeft,0))
                .build());
    }

    // a basic action like this returns false immediately so the runBlocking moves on to the next action while this completes
    public class ServoAction implements Action {
        Servo servo;
        double position;

        public ServoAction(Servo s, double p) {
            this.servo = s;
            this.position = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            servo.setPosition(position);
            return false; // returning true causes the action to run again, returning false causes it to cease
        }
    }

    // This doesn't immediately return false - it keeps running until the return statement is false
    // (i.e. it's been running for 3 seconds or motor has gotten to position).
    // Thus the runBlocking action doesn't move on until this one is done
    public class PatientServoAction implements Action {
        Servo servo;
        double position;
        ElapsedTime timer;

        boolean hasInitialized;

        public PatientServoAction(Servo s, double p) {
            this.servo = s;
            this.position = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!hasInitialized){
                timer = new ElapsedTime();
                servo.setPosition(position);
            }

            // do we need to keep running?
            return timer.seconds() < 3;
            // or return motor.getPosition == targetPosition;
        }
    }
}

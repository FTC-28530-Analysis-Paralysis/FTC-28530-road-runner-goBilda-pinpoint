package org.firstinspires.ftc.teamcode.in_development;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Autonomous(name="RR Auto Action Testing", group = "In_Development", preselectTeleOp = "CompetitionTeleopModified")
//@Disabled
public class RRAutoActionTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        CRServo intake = hardwareMap.crservo.get("intake");
        DcMotor slide = hardwareMap.dcMotor.get("slide");

        waitForStart();

        Actions.runBlocking(
            drive.actionBuilder(new Pose2d(0,0,0))
                .lineToX(20)
                .stopAndAdd(new PatientServoAction(intake, 1))
                .lineToX(15)
                .stopAndAdd(new MotorRunToPositionAction(slide, 100, 1000))
                .lineToX(10)
                .stopAndAdd(new WaitUntilMotorDoneAction(slide))
                .lineToX(20)
                .stopAndAdd(new MotorRunToPositionAction(slide, 10, 1000))
                .stopAndAdd(new WaitUntilMotorDoneAction(slide))
                .build());
    }

    public class ServoAction implements Action {
        CRServo crServo;
        double power;

        public ServoAction(CRServo s, double pos) {
            this.crServo = s;
            this.power = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            crServo.setPower(power);
            return false;
        }
    }

    public class PatientServoAction implements Action {
        CRServo crServo;
        double power;
        ElapsedTime timer;

        public PatientServoAction(CRServo s, double pos) {
            this.crServo = s;
            this.power = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null){
                timer = new ElapsedTime();
                crServo.setPower(power);
            }

            // do we need to keep running?
            return timer.seconds() < 3;
        }
    }

    public class MotorRunToPositionAction implements Action {
        DcMotor motor;
        int position;
        int motorVelocity;

        public MotorRunToPositionAction(DcMotor m, int position, int motorVelocity) {
            this.motor = m;
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            motor.setTargetPosition(position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(.5);
            return false;
        }
    }

    public class WaitUntilMotorDoneAction implements Action {
        DcMotor motor;

        public WaitUntilMotorDoneAction(DcMotor m) {
            this.motor = m;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return !motor.isBusy();
        }
    }

}

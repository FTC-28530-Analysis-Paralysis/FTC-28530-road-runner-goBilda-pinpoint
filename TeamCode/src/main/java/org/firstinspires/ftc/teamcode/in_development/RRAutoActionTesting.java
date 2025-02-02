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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Autonomous(name="RR Auto Action Testing", group = "In_Development")
//@Disabled
public class RRAutoActionTesting extends LinearOpMode {
    public DcMotor armMotor = null;
    public DcMotor slideMotor = null;
    public CRServo intake = null;
    public Servo wrist = null;

    final double INTAKE_OFF = 0.0;
    final double WRIST_FOLDED_OUT = 0.2;

    final double SLIDE_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation

    final int ARM_COLLAPSED_INTO_ROBOT  = 0;
    final int ARM_COLLECT               = (int) (5   * ARM_TICKS_PER_DEGREE);
    final int ARM_CLEAR_BARRIER         = (int) (15  * ARM_TICKS_PER_DEGREE);
    final int ARM_SCORE_SPECIMEN        = (int) (90  * ARM_TICKS_PER_DEGREE);
    final int ARM_SCORE_SAMPLE_IN_HIGH  = (int) (99  * ARM_TICKS_PER_DEGREE);
    final int ARM_ATTACH_HANGING_HOOK   = (int) (106 * ARM_TICKS_PER_DEGREE);
    final int ARM_HANG                  = (int) (10  * ARM_TICKS_PER_DEGREE);

    final int SLIDE_COLLAPSED = 0;
    final int SLIDE_SCORING_IN_LOW_BASKET = 0;
    final int SLIDE_SCORING_IN_HIGH_BASKET = (int) (420 * SLIDE_TICKS_PER_MM);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        intake = hardwareMap.crservo.get("intake");
        slideMotor = hardwareMap.dcMotor.get("slide");
        armMotor = hardwareMap.dcMotor.get("arm");
        wrist = hardwareMap.servo.get("wrist");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake.setPower(INTAKE_OFF);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        wrist.setPosition(WRIST_FOLDED_OUT);

        Actions.runBlocking(
            drive.actionBuilder(new Pose2d(37,61.7,Math.toRadians(-90)))
                .setTangent(Math.toRadians(0))
                .lineToX(10)
                .stopAndAdd(new MotorRunToPositionAction(armMotor, ARM_SCORE_SPECIMEN,1000))
                .stopAndAdd(new MotorRunToPositionAction(slideMotor, SLIDE_SCORING_IN_HIGH_BASKET, 1000))
                .lineToX(0)
                .stopAndAdd(new WaitUntilMotorDoneAction(slideMotor))
                .stopAndAdd(new WaitUntilMotorDoneAction(armMotor))
                .stopAndAdd(new PatientServoAction(intake,.5))
                .lineToX(10)
                .stopAndAdd(new MotorRunToPositionAction(armMotor, ARM_ATTACH_HANGING_HOOK, 1000))
                .stopAndAdd(new MotorRunToPositionAction(slideMotor, SLIDE_COLLAPSED, 1000))
                .stopAndAdd(new WaitUntilMotorDoneAction(slideMotor))
                .stopAndAdd(new WaitUntilMotorDoneAction(armMotor))
                .waitSeconds(3)
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

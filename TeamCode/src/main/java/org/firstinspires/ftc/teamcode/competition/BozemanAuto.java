package org.firstinspires.ftc.teamcode.competition;

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
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointDrive;

@Autonomous(name="RR Auto Action Testing", group = "In_Development")
//@Disabled
public class BozemanAuto extends LinearOpMode {

    private PinpointDrive drive;
    private Pose2d initialPose;
    public DcMotor armMotor = null;
    public DcMotor slideMotor = null;
    public CRServo intake = null;
    public Servo wrist = null;

    final double INTAKE_OFF = 0.0;
    final double INTAKE_COLLECT = 1.0;
    final double INTAKE_DEPOSIT = -0.5;
    final double WRIST_FOLDED_OUT = 0;
    final double WRIST_FOLDED_IN = 1;

    final double SLIDE_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation

    final int ARM_COLLAPSED_INTO_ROBOT  = 0;
    final int ARM_COLLECT               = (int) (5   * ARM_TICKS_PER_DEGREE);
    final int ARM_CLEAR_BARRIER         = (int) (15  * ARM_TICKS_PER_DEGREE);
    final int ARM_ABOVE_HIGH_RUNG       = (int) (93  * ARM_TICKS_PER_DEGREE);
    final int ARM_SCORE_SPECIMEN        = (int) (70  * ARM_TICKS_PER_DEGREE);
    final int ARM_SCORE_SAMPLE_IN_HIGH  = (int) (99  * ARM_TICKS_PER_DEGREE);
    final int ARM_ATTACH_HANGING_HOOK   = (int) (106 * ARM_TICKS_PER_DEGREE);
    final int ARM_HANG                  = (int) (10  * ARM_TICKS_PER_DEGREE);
    final int ARM_TOUCH_BAR             = (int) (90  * ARM_TICKS_PER_DEGREE);

    final int ARM_VELOCITY = 2100;
    final int SLIDE_VELOCITY = 2100;

    final int SLIDE_COLLAPSED = 0;
    final int SLIDE_SCORE_SPECIMEN = (int) (10 * SLIDE_TICKS_PER_MM);
    final int SLIDE_SCORING_IN_LOW_BASKET = 0;
    final int SLIDE_TOUCH_BAR = (int) (50 * SLIDE_TICKS_PER_MM);
    final int SLIDE_SCORING_IN_HIGH_BASKET = (int) (400 * SLIDE_TICKS_PER_MM);

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
        wrist.setPosition(WRIST_FOLDED_IN);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        initialPose = new Pose2d(7.25, 61.7, Math.toRadians(-90));
        drive = new PinpointDrive(hardwareMap, initialPose);
        Actions.runBlocking(
            drive.actionBuilder(initialPose)
                    // start - swing arm to score specimen position and move toward high rung
                    .stopAndAdd(new MotorRunToPositionAction(armMotor, ARM_ABOVE_HIGH_RUNG, ARM_VELOCITY))
                    .lineToY(40)
                    .stopAndAdd(new WaitUntilMotorDoneAction(armMotor))
                    .lineToY(32)
                    // lower arm until specimen is hooked on high rung, reverse intake, back away
                    .stopAndAdd(new MotorRunToPositionAction(armMotor, ARM_SCORE_SPECIMEN, ARM_VELOCITY))
                    .stopAndAdd(new WaitUntilMotorDoneAction(armMotor))
                    .waitSeconds(.2)
                    .lineToY(40)

                    // Move to sample 1
                    .stopAndAdd(new MotorRunToPositionAction(slideMotor, SLIDE_COLLAPSED, SLIDE_VELOCITY))
                    .stopAndAdd(new ServoAction(wrist, WRIST_FOLDED_OUT))
                    .stopAndAdd(new MotorRunToPositionAction(armMotor, ARM_COLLECT, ARM_VELOCITY))
                    .setTangent(Math.toRadians(0))
                    .splineToLinearHeading(new Pose2d(29.125,27.5,Math.toRadians(0)), Math.toRadians(0))
                    .setTangent(Math.toRadians(-90))
                    .lineToY(20.5)
                    // Intake sample 1
                    .stopAndAdd(new CRServoAction(intake, INTAKE_COLLECT))
                    .lineToX(32)
                    .waitSeconds(1.2)
                    .stopAndAdd(new CRServoAction(intake, INTAKE_OFF))

                    // Move to Basket
                    .stopAndAdd(new MotorRunToPositionAction(armMotor, ARM_SCORE_SAMPLE_IN_HIGH, ARM_VELOCITY))
                    .waitSeconds( .7)
                    .stopAndAdd(new MotorRunToPositionAction(slideMotor, SLIDE_SCORING_IN_HIGH_BASKET, SLIDE_VELOCITY))
                    .setTangent(Math.toRadians(45))
                    .splineToLinearHeading(new Pose2d(51,50,Math.toRadians(45)), Math.toRadians(45))
                    // Deposit sample 1
                    .stopAndAdd(new WaitUntilMotorDoneAction(armMotor))
                    .stopAndAdd(new WaitUntilMotorDoneAction(slideMotor))
                    .setTangent(Math.toRadians(45))
                    .lineToX(48)
                    .stopAndAdd(new CRServoAction(intake, INTAKE_DEPOSIT))
                    .waitSeconds(.5)
                    .stopAndAdd(new CRServoAction(intake, INTAKE_OFF))

                    // Move to sample 2
                    .stopAndAdd(new MotorRunToPositionAction(slideMotor, SLIDE_COLLAPSED, SLIDE_VELOCITY))
                    .waitSeconds(.5)
                    .stopAndAdd(new MotorRunToPositionAction(armMotor, ARM_COLLECT, ARM_VELOCITY))
                    .setTangent(Math.toRadians(-135))
                    .splineToLinearHeading(new Pose2d(39.5,28,Math.toRadians(0)), Math.toRadians(0))
                    .setTangent(Math.toRadians(-90))
                    .lineToY(20)
                    // Intake sample 2
                    .stopAndAdd(new CRServoAction(intake, INTAKE_COLLECT))
                    .stopAndAdd(new WaitUntilMotorDoneAction(armMotor))
                    .stopAndAdd(new WaitUntilMotorDoneAction(slideMotor))
                    .setTangent(Math.toRadians(0))
                    .lineToX(42.5)
                    .waitSeconds(.5)
                    .stopAndAdd(new CRServoAction(intake, INTAKE_OFF))

                     //Move to Basket
                    .stopAndAdd(new MotorRunToPositionAction(armMotor, ARM_SCORE_SAMPLE_IN_HIGH, ARM_VELOCITY))
                    .waitSeconds(.7)
                    .stopAndAdd(new MotorRunToPositionAction(slideMotor, SLIDE_SCORING_IN_HIGH_BASKET, SLIDE_VELOCITY))
                    .setTangent(Math.toRadians(45))
                    .splineToLinearHeading(new Pose2d(51,50,Math.toRadians(45)), Math.toRadians(45))
                    // Deposit sample 2
                    //.stopAndAdd(new WaitUntilMotorDoneAction(armMotor))
                    //.stopAndAdd(new WaitUntilMotorDoneAction(slideMotor))
                    .waitSeconds(.2)
                    .setTangent(Math.toRadians(45))
                    .lineToX(48)
                    .stopAndAdd(new CRServoAction(intake, INTAKE_DEPOSIT))
                    .waitSeconds(.5)
                    .stopAndAdd(new CRServoAction(intake, INTAKE_OFF))

                    // Move to sample 3
                    .stopAndAdd(new MotorRunToPositionAction(slideMotor, SLIDE_COLLAPSED, SLIDE_VELOCITY))
                    .waitSeconds(.5)
                    .stopAndAdd(new MotorRunToPositionAction(armMotor, ARM_COLLECT, ARM_VELOCITY))
                    .setTangent(Math.toRadians(-135))
                    .splineToLinearHeading(new Pose2d(49,20,Math.toRadians(0)), Math.toRadians(0))
                    // Intake sample 3
                    //.stopAndAdd(new WaitUntilMotorDoneAction(armMotor))
                    //.stopAndAdd(new WaitUntilMotorDoneAction(slideMotor))
                    .waitSeconds(.2)
                    .stopAndAdd(new CRServoAction(intake, INTAKE_COLLECT))
                    .lineToX(52)
                    .waitSeconds(.5)
                    .stopAndAdd(new CRServoAction(intake, INTAKE_OFF))

                    // Move to Basket
                    .setTangent(Math.toRadians(45))
                    .splineToLinearHeading(new Pose2d(49,20,Math.toRadians(45)), Math.toRadians(45))
                    .stopAndAdd(new MotorRunToPositionAction(armMotor, ARM_SCORE_SAMPLE_IN_HIGH, ARM_VELOCITY))
                    .waitSeconds(.7)
                    .stopAndAdd(new MotorRunToPositionAction(slideMotor, SLIDE_SCORING_IN_HIGH_BASKET, SLIDE_VELOCITY))
                    .setTangent(Math.toRadians(45))
                    .splineToLinearHeading(new Pose2d(53,50,Math.toRadians(45)), Math.toRadians(45))
                    // Deposit sample 3
                    //.stopAndAdd(new WaitUntilMotorDoneAction(armMotor))
                    //.stopAndAdd(new WaitUntilMotorDoneAction(slideMotor))
                    .waitSeconds(.2)
                    .setTangent(Math.toRadians(45))
                    .lineToX(48)
                    .stopAndAdd(new CRServoAction(intake, INTAKE_DEPOSIT))
                    .waitSeconds(.5)

//                    // Move to ascent
//                    .stopAndAdd(new MotorRunToPositionAction(armMotor, ARM_TOUCH_BAR, ARM_VELOCITY))
//                    .stopAndAdd(new MotorRunToPositionAction(slideMotor, SLIDE_TOUCH_BAR, SLIDE_VELOCITY))
//                    .setTangent(-90)
//                    .splineToLinearHeading(new Pose2d(45, 30, Math.toRadians(-90)), Math.toRadians(-110))
//                    .splineToLinearHeading(new Pose2d(30,12, Math.toRadians(180)), Math.toRadians(180))
//                    // Touch Arm to bar
//                    .stopAndAdd(new WaitUntilMotorDoneAction(armMotor))
//                    .stopAndAdd(new WaitUntilMotorDoneAction(slideMotor))
//                    .lineToX(22)
                    .build());

        if (!opModeIsActive()){
            BozemanTeleop.endOfAutoArmPosition = armMotor.getCurrentPosition();
            BozemanTeleop.endOfAutoSlidePosition = slideMotor.getCurrentPosition();
        }
    }

    public class CRServoAction implements Action {
        CRServo crServo;
        double power;

        public CRServoAction(CRServo s, double pos) {
            this.crServo = s;
            this.power = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            crServo.setPower(power);
            return false;
        }
    }
    public class ServoAction implements Action {
        Servo servo;
        double position;

        public ServoAction(Servo s, double pos) {
            this.servo = s;
            this.position = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            servo.setPosition(position);
            return false;
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
            motor.setPower(1);
            return false;
        }
    }

    public class WaitUntilMotorDoneAction implements Action {
        DcMotor motor;
        ElapsedTime timer;
        int position;

        public WaitUntilMotorDoneAction(DcMotor m) {
            this.motor = m;
            this.timer = new ElapsedTime();
            this.position = motor.getCurrentPosition();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null){
                timer = new ElapsedTime();
            }
            if (timer.seconds() > 2) return true;
            else return Math.abs(motor.getCurrentPosition() - position) < 10;
        }
    }

}

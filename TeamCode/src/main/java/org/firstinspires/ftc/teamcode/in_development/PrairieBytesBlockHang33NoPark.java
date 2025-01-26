package org.firstinspires.ftc.teamcode.in_development;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "PrairieBytesBlockHang33NoPark", preselectTeleOp = "PB-Dev-V3.3")
public class PrairieBytesBlockHang33NoPark extends LinearOpMode {

    private DcMotor LiftArm;
    private DcMotor Slider;
    private DcMotor LeftFront;
    private DcMotor LeftRear;
    private DcMotor RightFront;
    private DcMotor RightRear;
    private VoltageSensor ControlHub_VoltageSensor;
    private DistanceSensor DistanceSensor_DistanceSensor;
    private Servo Wrist;
    private Servo RangeServo;

    boolean TagDectected;
    int armPosition;
    boolean USE_WEBCAM;
    int Mode;
    double WRIST_FOLDED_IN;
    String currentDirection;
    double rangeSrvSide;
    double rangeSrvBack;
    double rangeSrvFwd;
    int ArmFolded;
    boolean ChangePosition;
    int MotorPostition;
    int setDistance;
    int setArm;
    double WRIST_FOLDED_UP;
    double sensorDistance;
    AprilTagProcessor myAprilTagProcessor;
    int delayBtwnMove_Read;
    double TagBearing;
    double TagDist;
    double TagYaw;
    double PositionLR;

    /**
     * Describe this function...
     */
    private void Init_Motors() {
        LiftArm.setDirection(DcMotor.Direction.FORWARD);
        LiftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftArm.setTargetPosition(0);
        LiftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) LiftArm).setVelocity(1000);
        // Slider
        Slider.setDirection(DcMotor.Direction.REVERSE);
        Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slider.setTargetPosition(0);
        Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) Slider).setVelocity(800);
        // LeftFront
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setTargetPosition(0);
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) LeftFront).setVelocity(1000);
        // LeftRear
        LeftRear.setDirection(DcMotor.Direction.REVERSE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftRear.setTargetPosition(0);
        LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) LeftRear).setVelocity(1000);
        // RightFront
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setTargetPosition(0);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) RightFront).setVelocity(1000);
        // RightRear
        RightRear.setDirection(DcMotor.Direction.FORWARD);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightRear.setTargetPosition(0);
        RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) RightRear).setVelocity(1000);
    }

    /**
     * Describe this function...
     */
    private void Init() {
        boolean FirstRun;
        boolean atDistance;
        int FrontLeftPos;
        int BackLeftPos;
        int FrontRightPos;
        int BackRightPos;
        int leftFrontPower;
        int leftBackPower;
        int rightFrontPower;
        int rightBackPower;
        int axial;
        int lateral;
        int yaw;
        int GoFwd;
        double WRIST_FOLDED_OUT;
        boolean Alined;

        // Initialize Motors
        Init_Motors();
        // Initialize Variables
        currentDirection = "";
        FirstRun = true;
        atDistance = false;
        FrontLeftPos = 0;
        BackLeftPos = 0;
        FrontRightPos = 0;
        BackRightPos = 0;
        leftFrontPower = 0;
        leftBackPower = 0;
        rightFrontPower = 0;
        rightBackPower = 0;
        sensorDistance = 0;
        rangeSrvFwd = 0.155;
        rangeSrvSide = 0.49;
        rangeSrvBack = 0.82;
        TagBearing = 0;
        TagDist = 0;
        TagYaw = 0;
        PositionLR = 0;
        axial = 0;
        lateral = 0;
        yaw = 0;
        delayBtwnMove_Read = 0;
        GoFwd = 1;
        ArmFolded = 1;
        WRIST_FOLDED_UP = 0.595;
        WRIST_FOLDED_IN = 0.595;
        WRIST_FOLDED_OUT = 0.925;
        Alined = false;
        ChangePosition = true;
        MotorPostition = 0;
        Mode = 0;
        // Telem Init
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    @Override
    public void runOpMode() {
        LiftArm = hardwareMap.get(DcMotor.class, "LiftArm");
        Slider = hardwareMap.get(DcMotor.class, "Slider");
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftRear = hardwareMap.get(DcMotor.class, "LeftRear");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightRear = hardwareMap.get(DcMotor.class, "RightRear");
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        DistanceSensor_DistanceSensor = hardwareMap.get(DistanceSensor.class, "DistanceSensor");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        RangeServo = hardwareMap.get(Servo.class, "RangeServo");

        // Initialization
        USE_WEBCAM = false;
        // Set backup distance
        setDistance = 550;
        // Set the arm hanging height
        setArm = 1420;
        Init();
        waitForStart();
        armPosition = 200;
        LiftArm.setTargetPosition(armPosition);
        sleep(500);
        while (opModeIsActive() && ControlHub_VoltageSensor.getVoltage() > 10) {
            sensorDistance = DistanceSensor_DistanceSensor.getDistance(DistanceUnit.INCH);
            Wrist.setPosition(WRIST_FOLDED_IN);
            Slider.setTargetPosition(0);
            if (Mode == 0) {
                MoveBkwd();
            } else if (Mode == 1) {
                UnfoldArm();
                ReadRangeFwd();
            } else if (Mode == 2) {
                EncBackup();
            } else if (Mode == 3) {
                Hang_Backup();
            } else if (Mode == 4) {
                Foldup();
            } else if (Mode == 5) {
                TurnAround();
            } else if (Mode == 6) {
            } else if (Mode == 7) {
            } else if (Mode == 8) {
            } else if (Mode == 9) {
            }
            MaxArm();
            LiftArm.setTargetPosition(armPosition);
            Telemetry2();
            telemetry.update();
        }
    }

    /**
     * Runtime telemetry
     */
    private void Telemetry2() {
        telemetry.addData("LiftArmPos", LiftArm.getCurrentPosition());
        telemetry.addData("LeftFrontPos", LeftFront.getCurrentPosition());
        telemetry.addData("RangeServoPos", RangeServo.getPosition());
        telemetry.addData("Direction", currentDirection);
        telemetry.addData("Range", sensorDistance);
        telemetry.addData("TagDetected", TagDectected);
        telemetry.addData("Mode", Mode);
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void ReadRangeSide() {
        if (!currentDirection.equals("Side")) {
            RangeServo.setPosition(rangeSrvSide);
            currentDirection = "Side";
            sleep(delayBtwnMove_Read);
        }
    }

    /**
     * Describe this function...
     */
    private void ReadRangeBack() {
        if (!currentDirection.equals("Back")) {
            RangeServo.setPosition(rangeSrvBack);
            currentDirection = "Back";
            sleep(delayBtwnMove_Read);
        }
    }

    /**
     * Describe this function...
     */
    private void ReadRangeFwd() {
        if (!currentDirection.equals("Forward")) {
            RangeServo.setPosition(rangeSrvFwd);
            currentDirection = "Forward";
            sleep(delayBtwnMove_Read);
        }
    }

    /**
     * Describe this function...
     */
    private void MaxArm() {
        if (armPosition < 0) {
            armPosition = 0;
        } else if (armPosition > 5000) {
            armPosition = 5000;
        }
    }

    /**
     * Describe this function...
     */
    private void UnfoldArm() {
        if (1 == ArmFolded) {
            if (LiftArm.getCurrentPosition() < 495) {
                armPosition = 500;
            } else {
                armPosition = 1700;
                if (550 < LiftArm.getCurrentPosition() && !LiftArm.isBusy()) {
                    ArmFolded = 0;
                    Mode = 2;
                }
            }
        }
    }

    /**
     * Describe this function...
     */
    private void MoveBkwd() {
        LeftFront.setTargetPosition(-750);
        LeftRear.setTargetPosition(-750);
        RightFront.setTargetPosition(-750);
        RightRear.setTargetPosition(-750);
        if (!(LeftFront.isBusy() || LeftRear.isBusy() || RightFront.isBusy() || RightRear.isBusy())) {
            Mode = 1;
            ResetMotors();
        }
    }

    /**
     * Describe this function...
     */
    private void ResetMotors() {
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setTargetPosition(0);
        LeftRear.setTargetPosition(0);
        RightFront.setTargetPosition(0);
        RightRear.setTargetPosition(0);
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) LeftFront).setVelocity(800);
        ((DcMotorEx) LeftRear).setVelocity(800);
        ((DcMotorEx) RightFront).setVelocity(800);
        ((DcMotorEx) RightRear).setVelocity(800);
    }

    /**
     * Describe this function...
     */
    private void EncBackup() {
        LeftFront.setTargetPosition(-setDistance);
        LeftRear.setTargetPosition(-setDistance);
        RightFront.setTargetPosition(-setDistance);
        RightRear.setTargetPosition(-setDistance);
        if (!(LeftFront.isBusy() || LeftRear.isBusy() || RightFront.isBusy() || RightRear.isBusy())) {
            Mode = 3;
            ResetMotors();
        }
    }

    /**
     * Describe this function...
     */
    private void Hang_Backup() {
        Wrist.setPosition(WRIST_FOLDED_IN);
        armPosition = setArm;
        LiftArm.setTargetPosition(armPosition);
        sleep(1000);
        LeftFront.setTargetPosition(225);
        LeftRear.setTargetPosition(225);
        RightFront.setTargetPosition(225);
        RightRear.setTargetPosition(225);
        if (!(LeftFront.isBusy() || LeftRear.isBusy() || RightFront.isBusy() || RightRear.isBusy())) {
            Mode = 4;
            ResetMotors();
        }
    }

    /**
     * Describe this function...
     */
    private void Foldup() {
        armPosition = 100;
        Mode = 5;
    }

    /**
     * Describe this function...
     */
    private void TurnAround() {
        LeftFront.setTargetPosition(-1950);
        LeftRear.setTargetPosition(-1950);
        RightFront.setTargetPosition(1950);
        RightRear.setTargetPosition(1950);
        ReadRangeBack();
        if (!(LeftFront.isBusy() || LeftRear.isBusy() || RightFront.isBusy() || RightRear.isBusy())) {
            Mode = 6;
            ResetMotors();
        }
    }

    /**
     * Describe this function...
     */
    private void MoveToWall() {
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (sensorDistance > 12) {
            LeftFront.setPower(-0.3);
            LeftRear.setPower(-0.3);
            RightFront.setPower(-0.3);
            RightRear.setPower(-0.3);
        } else {
            LeftFront.setPower(0);
            LeftRear.setPower(0);
            RightFront.setPower(0);
            RightRear.setPower(0);
            Mode = 7;
            ReadRangeSide();
        }
    }

    /**
     * Describe this function...
     */
    private void MoveToCorner() {
        if (sensorDistance > 6) {
            LeftFront.setPower(0.3);
            LeftRear.setPower(-0.3);
            RightFront.setPower(-0.3);
            RightRear.setPower(0.27);
        } else {
            LeftFront.setPower(0);
            LeftRear.setPower(0);
            RightFront.setPower(0);
            RightRear.setPower(0);
            Mode = 8;
            ResetMotors();
        }
    }

    /**
     * Describe this function...
     */
    private void Turn() {
        RightFront.setTargetPosition(-975);
        RightRear.setTargetPosition(-975);
        LeftFront.setTargetPosition(975);
        LeftRear.setTargetPosition(975);
        ReadRangeSide();
        sleep(3000);
        ResetMotors();
        Mode = 9;
    }

    /**
     * Describe this function...
     */
    private void StrafetoCorner() {
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (sensorDistance > 4) {
            LeftFront.setPower(0.22);
            LeftRear.setPower(-0.2);
            RightFront.setPower(-0.25);
            RightRear.setPower(0.2);
        } else {
            LeftFront.setPower(0);
            LeftRear.setPower(0);
            RightFront.setPower(0);
            RightRear.setPower(0);
            Mode = 10;
            ResetMotors();
        }
    }
}
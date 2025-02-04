package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.106975383400094)
                .build();

        bot1.runAction(bot1.getDrive().actionBuilder(new Pose2d(6, 61.7, Math.toRadians(-90)))
                                .setTangent(0)
                                .splineTo(new Vector2d(61,45),0)
                                .setTangent(Math.toRadians(90))
                // start - swing arm to score specimen position and move toward high rung
                //.stopAndAdd(new MotorRunToPositionAction(armMotor, ARM_ABOVE_HIGH_RUNG, ARM_VELOCITY))
                .lineToY(35)
                // lower arm until specimen is hooked on high rung, reverse intake, back away
                //.stopAndAdd(new MotorRunToPositionAction(armMotor, ARM_SCORE_SPECIMEN, ARM_VELOCITY))
                //.stopAndAdd(new WaitUntilMotorDoneAction(armMotor))
                //.stopAndAdd(new CRServoAction(intake, INTAKE_DEPOSIT))
                .waitSeconds(.5)

                // Move to sample 1
                //.stopAndAdd(new MotorRunToPositionAction(armMotor, ARM_COLLECT, ARM_VELOCITY))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(40,35,Math.toRadians(-45)), Math.toRadians(-45))
                //.stopAndAdd(new WaitUntilMotorDoneAction(armMotor))
                // Intake sample 1
                //.stopAndAdd(new CRServoAction(intake, INTAKE_COLLECT))
                .setTangent(Math.toRadians(-45))
                .lineToX(42)
                .waitSeconds(.5)
                //.stopAndAdd(new CRServoAction(intake, INTAKE_OFF))

                // Move to Basket
                //.stopAndAdd(new MotorRunToPositionAction(armMotor, ARM_SCORE_SAMPLE_IN_HIGH, ARM_VELOCITY))
                //.stopAndAdd(new MotorRunToPositionAction(slideMotor, SLIDE_SCORING_IN_HIGH_BASKET, SLIDE_VELOCITY))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(53,50,Math.toRadians(45)), Math.toRadians(45))
                // Deposit sample 1
                //.stopAndAdd(new WaitUntilMotorDoneAction(armMotor))
                //.stopAndAdd(new WaitUntilMotorDoneAction(slideMotor))
                .setTangent(Math.toRadians(45))
                .lineToX(55)
                //.stopAndAdd(new CRServoAction(intake, INTAKE_DEPOSIT))
                .waitSeconds(.5)

                // Move to sample 2
                //.stopAndAdd(new MotorRunToPositionAction(slideMotor, SLIDE_COLLAPSED, SLIDE_VELOCITY))
                .waitSeconds(.5)
                //.stopAndAdd(new MotorRunToPositionAction(armMotor, ARM_COLLECT, ARM_VELOCITY))
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(48,35,Math.toRadians(-45)), Math.toRadians(-45))
                // Intake sample 2
                //.stopAndAdd(new WaitUntilMotorDoneAction(armMotor))
                //.stopAndAdd(new WaitUntilMotorDoneAction(slideMotor))
                //.stopAndAdd(new CRServoAction(intake, INTAKE_COLLECT))
                .setTangent(Math.toRadians(-45))
                .lineToX(50)
                .waitSeconds(.5)
                //.stopAndAdd(new CRServoAction(intake, INTAKE_OFF))

                // Move to Basket
                //.stopAndAdd(new MotorRunToPositionAction(armMotor, ARM_SCORE_SAMPLE_IN_HIGH, ARM_VELOCITY))
                //.stopAndAdd(new MotorRunToPositionAction(slideMotor, SLIDE_SCORING_IN_HIGH_BASKET, SLIDE_VELOCITY))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(53,50,Math.toRadians(45)), Math.toRadians(45))
                // Deposit sample 2
                //.stopAndAdd(new WaitUntilMotorDoneAction(armMotor))
                //.stopAndAdd(new WaitUntilMotorDoneAction(slideMotor))
                .setTangent(Math.toRadians(45))
                .lineToX(55)
                //.stopAndAdd(new CRServoAction(intake, INTAKE_DEPOSIT))
                .waitSeconds(.5)

                // Move to sample 3
                //.stopAndAdd(new MotorRunToPositionAction(slideMotor, SLIDE_COLLAPSED, SLIDE_VELOCITY))
                .waitSeconds(.5)
                //.stopAndAdd(new MotorRunToPositionAction(armMotor, ARM_COLLECT, ARM_VELOCITY))
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(58,25,Math.toRadians(0)), Math.toRadians(0))
                // Intake sample 3
                //.stopAndAdd(new WaitUntilMotorDoneAction(armMotor))
                //.stopAndAdd(new WaitUntilMotorDoneAction(slideMotor))
                //.stopAndAdd(new CRServoAction(intake, INTAKE_COLLECT))
                .lineToX(60)
                .waitSeconds(.5)
                //.stopAndAdd(new CRServoAction(intake, INTAKE_OFF))

                // Move to Basket
                //.stopAndAdd(new MotorRunToPositionAction(armMotor, ARM_SCORE_SAMPLE_IN_HIGH, ARM_VELOCITY))
                //.stopAndAdd(new MotorRunToPositionAction(slideMotor, SLIDE_SCORING_IN_HIGH_BASKET, SLIDE_VELOCITY))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(53,50,Math.toRadians(45)), Math.toRadians(45))
                // Deposit sample 3
                //.stopAndAdd(new WaitUntilMotorDoneAction(armMotor))
                //.stopAndAdd(new WaitUntilMotorDoneAction(slideMotor))
                .setTangent(Math.toRadians(45))
                .lineToX(55)
                //.stopAndAdd(new CRServoAction(intake, INTAKE_DEPOSIT))
                .waitSeconds(.5)

                // Move to ascent
                //.stopAndAdd(new MotorRunToPositionAction(armMotor, ARM_TOUCH_BAR, ARM_VELOCITY))
                //.stopAndAdd(new MotorRunToPositionAction(slideMotor, SLIDE_TOUCH_BAR, SLIDE_VELOCITY))
                .setTangent(-90)
                .splineToLinearHeading(new Pose2d(45, 30, Math.toRadians(-90)), Math.toRadians(-110))
                .splineToLinearHeading(new Pose2d(30,12, Math.toRadians(180)), Math.toRadians(180))
                // Touch Arm to bar
                //.stopAndAdd(new WaitUntilMotorDoneAction(armMotor))
                //.stopAndAdd(new WaitUntilMotorDoneAction(slideMotor))
                .lineToX(22)
                .build());

        RoadRunnerBotEntity bot2 = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.106975383400094)
                .build();

        bot2.runAction(bot2.getDrive().actionBuilder(new Pose2d(-11.8, 61.7, Math.toRadians(-90)))
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

                meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot1)
                .addEntity(bot2)
                .start();
    }
}
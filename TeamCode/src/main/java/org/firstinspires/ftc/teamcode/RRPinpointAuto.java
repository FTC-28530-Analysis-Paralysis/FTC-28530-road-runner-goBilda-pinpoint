package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "RR Pinpoint Auto", group = "RoadRunner")
@Disabled
public class RRPinpointAuto extends OpMode {

    @Override
    public void init() {
        // instantiate the MecanumDrive at a particular pose.
        Pose2d intialPose = new Pose2d(-30, -60, Math.toRadians(90));
    }

    @Override
    public void loop() {

    }
}

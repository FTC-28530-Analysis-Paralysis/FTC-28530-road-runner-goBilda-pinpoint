package org.firstinspires.ftc.teamcode.in_development;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

@TeleOp(name = "PrairieBytesColorLocator")
public class PrairieBytesColorLocator extends LinearOpMode {

    private DcMotor LiftArm;
    private DcMotor Slider;
    private DcMotor LeftFront;
    private DcMotor LeftRear;
    private DcMotor RightFront;
    private DcMotor RightRear;
    private CRServo GrabberWheel;
    private Servo Wrist;

    int Mode;
    int armPosition;
    List<ColorBlobLocatorProcessor.Blob> myBlobs;
    double WRIST_FOLDED_OUT;
    ColorBlobLocatorProcessor myColorBlobLocatorProcessor;
    boolean BlobDetected;
    int armExtention;
    double PositionLR;

    /**
     * Describe this function...
     */
    private void InitColorLocator() {
        ColorBlobLocatorProcessor.Builder myColorBlobLocatorProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;
        VisionPortal myVisionPortal;

        // Build a "Color Locator" vision processor based on the ColorBlobLocatorProcessor class.
        myColorBlobLocatorProcessorBuilder = new ColorBlobLocatorProcessor.Builder();
        // - Specify the color range you are looking for.
        myColorBlobLocatorProcessorBuilder.setTargetColorRange(ColorRange.YELLOW);
        // - Focus the color locator by defining a RegionOfInterest (ROI) which you want to search.
        //     This can be the entire frame, or a sub-region defined using:
        //     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
        //     Use one form of the ImageRegion class to define the ROI.
        myColorBlobLocatorProcessorBuilder.setRoi(ImageRegion.entireFrame());
        // - Define which contours are included.
        //     You can get ALL the contours, or you can skip any contours that are completely inside another contour.
        //     note: EXTERNAL_ONLY helps to avoid bright reflection spots from breaking up areas of solid color.
        myColorBlobLocatorProcessorBuilder.setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY);
        // - Turn the display of contours ON or OFF.  Turning this on helps debugging but takes up valuable CPU time.
        myColorBlobLocatorProcessorBuilder.setDrawContours(true);
        // - Include any pre-processing of the image or mask before looking for Blobs.
        //     There are some extra processing you can include to improve the formation of blobs.  Using these features requires
        //     an understanding of how they may effect the final blobs.  The "pixels" argument sets the NxN kernel size.
        //     Blurring an image helps to provide a smooth color transition between objects, and smoother contours.
        //     The higher the number of pixels, the more blurred the image becomes.
        //     Note:  Even "pixels" values will be incremented to satisfy the "odd number" requirement.
        //     Blurring too much may hide smaller features.  A "pixels" size of 5 is good for a 320x240 image.
        myColorBlobLocatorProcessorBuilder.setBlurSize(5);
        //     Erosion removes floating pixels and thin lines so that only substantive objects remain.
        //     Erosion can grow holes inside regions, and also shrink objects.
        //     "pixels" in the range of 2-4 are suitable for low res images.
        //     Dilation makes objects more visible by filling in small holes, making lines appear thicker,
        //     and making filled shapes appear larger. Dilation is useful for joining broken parts of an
        //     object, such as when removing noise from an image.
        //     "pixels" in the range of 2-4 are suitable for low res images.
        myColorBlobLocatorProcessor = myColorBlobLocatorProcessorBuilder.build();
        // Build a vision portal to run the Color Locator process.
        myVisionPortalBuilder = new VisionPortal.Builder();
        //  - Add the ColorBlobLocatorProcessor created above.
        myVisionPortalBuilder.addProcessor(myColorBlobLocatorProcessor);
        //  - Set the desired video resolution.
        //      Since a high resolution will not improve this process, choose a lower resolution that is
        //      supported by your camera. This will improve overall performance and reduce latency.
        myVisionPortalBuilder.setCameraResolution(new Size(320, 240));
        //  - Choose your video source. This may be for a webcam or for a Phone Camera.
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        myVisionPortal = myVisionPortalBuilder.build();
    }

    /**
     * Describe this function...
     */
    private void IntiVariables() {
        double WRIST_FOLDED_IN;

        Mode = 0;
        WRIST_FOLDED_IN = 0.595;
        WRIST_FOLDED_OUT = 0.925;
        armPosition = 0;
        armExtention = 0;
        BlobDetected = false;
        PositionLR = 0;
    }

    /**
     * Describe this function...
     */
    private void Init_Motors() {
        LiftArm.setDirection(DcMotor.Direction.FORWARD);
        LiftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftArm.setTargetPosition(0);
        LiftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) LiftArm).setVelocity(400);
        // Slider
        Slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slider.setDirection(DcMotor.Direction.REVERSE);
        Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slider.setTargetPosition(armExtention);
        Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) Slider).setVelocity(1200);
        // LeftFront
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setTargetPosition(0);
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) LeftFront).setVelocity(400);
        // LeftRear
        LeftRear.setDirection(DcMotor.Direction.REVERSE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftRear.setTargetPosition(0);
        LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) LeftRear).setVelocity(400);
        // RightFront
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setTargetPosition(0);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) RightFront).setVelocity(400);
        // RightRear
        RightRear.setDirection(DcMotor.Direction.FORWARD);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightRear.setTargetPosition(0);
        RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) RightRear).setVelocity(400);
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
     * This OpMode illustrates how to use a video source (camera) to locate specifically colored regions.
     *
     * Unlike a "color sensor" which determines the color of an object in the
     * field of view, this "color locator" will search the Region Of Interest
     * (ROI) in a camera image, and find any "blobs" of color that match the
     * requested color range. These blobs can be further filtered and sorted
     * to find the one most likely to be the item the user is looking for.
     *
     * To perform this function, a VisionPortal runs a ColorBlobLocatorProcessor process.
     * The ColorBlobLocatorProcessor process is created first,
     * and then the VisionPortal is built to use this process.
     * The ColorBlobLocatorProcessor analyses the ROI and locates pixels that match the ColorRange to form a "mask".
     * The matching pixels are then collected into contiguous "blobs" of
     * pixels. The outer boundaries of these blobs are called its "contour".
     * For each blob, the process then creates the smallest possible
     * rectangle "boxFit" that will fully encase the contour.
     * The user can then call getBlobs() to retrieve the list of Blobs,
     * where each Blob contains the contour and the boxFit data.
     * Note: The default sort order for Blobs is ContourArea, in descending order, so the biggest contours are
     *   listed first.
     *
     * To aid the user, a colored boxFit rectangle is drawn on the camera preview to
     * show the location of each Blob. The original Blob contour can also be added to the
     * preview. This is helpful when configuring the ColorBlobLocatorProcessor parameters.
     */
    @Override
    public void runOpMode() {
        LiftArm = hardwareMap.get(DcMotor.class, "LiftArm");
        Slider = hardwareMap.get(DcMotor.class, "Slider");
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftRear = hardwareMap.get(DcMotor.class, "LeftRear");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightRear = hardwareMap.get(DcMotor.class, "RightRear");
        GrabberWheel = hardwareMap.get(CRServo.class, "GrabberWheel");
        Wrist = hardwareMap.get(Servo.class, "Wrist");

        InitColorLocator();
        IntiVariables();
        Init_Motors();
        // Speed up telemetry updates, Just use for debugging.
        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        LiftArm.setTargetPosition(armPosition);
        Slider.setTargetPosition(armExtention);
        waitForStart();
        while (opModeIsActive() || false) {
            telemetry.addData("preview on/off", "... Camera Stream");
            telemetry.addLine("");
            RunColorLocator();
            DisplayResults();
            if (Mode == 0) {
                Unfold();
            } else if (Mode == 1) {
                CenterOnBlock();
            } else if (Mode == 2) {
                Pickup();
            }
            LiftArm.setTargetPosition(armPosition);
            Slider.setTargetPosition(armExtention);
            Telemetry2();
            telemetry.update();
            sleep(50);
        }
    }

    /**
     * Describe this function...
     */
    private void Telemetry2() {
        telemetry.addData("Mode", Mode);
        telemetry.addData("armPosition", armPosition);
        telemetry.addData("armEncoder", LiftArm.getTargetPosition());
        telemetry.addData("armExtention", armExtention);
        telemetry.addData("sliderEncoder", Slider.getTargetPosition());
    }

    /**
     * Describe this function...
     */
    private void Pickup() {
        GrabberWheel.setPower(-0.5);
        LeftFront.setTargetPosition(-750);
        LeftRear.setTargetPosition(-750);
        RightFront.setTargetPosition(-750);
        RightRear.setTargetPosition(-750);
        if (!(LeftFront.isBusy() || LeftRear.isBusy() || RightFront.isBusy() || RightRear.isBusy())) {
            Mode = 3;
            ResetMotors();
            GrabberWheel.setPower(0);
        }
    }

    /**
     * Describe this function...
     */
    private void Unfold() {
        armPosition = 310;
        LiftArm.setTargetPosition(armPosition);
        sleep(1000);
        Wrist.setPosition(WRIST_FOLDED_OUT);
        sleep(500);
        armExtention = 1340;
        Slider.setTargetPosition(armExtention);
        sleep(1500);
        Mode = 1;
    }

    /**
     * Describe this function...
     */
    private void CenterOnBlock() {
        boolean atLateral = false;
        int yaw;
        int axial;
        int centered;
        boolean atYaw;
        double leftFrontPower;
        double TagBearing;
        double TagYaw;
        double lateral = 0;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;

        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftRear.setDirection(DcMotor.Direction.REVERSE);
        yaw = 0;
        axial = 0;
        centered = 0;
        while (Mode == 1 && opModeIsActive() && BlobDetected) {
            RunColorLocator();
            DisplayResults();
            telemetry.update();
            if (PositionLR > -15 && PositionLR < 15) {
                atLateral = true;
            }
            if (!atLateral) {
                if (PositionLR > 0) {
                    lateral = 0.2;
                } else {
                    lateral = -0.2;
                }
            }
            leftFrontPower = (axial - lateral) + yaw;
            leftBackPower = axial + lateral + yaw;
            rightFrontPower = (axial + lateral) - yaw;
            rightBackPower = (axial - lateral) - yaw;
            LeftFront.setPower(leftFrontPower);
            LeftRear.setPower(leftBackPower);
            RightFront.setPower(rightFrontPower);
            RightRear.setPower(rightBackPower);
            if (atLateral) {
                ResetMotors();
                Mode = 2;
            }
        }
    }

    /**
     * Describe this function...
     */
    private void RunColorLocator() {
        // Read the current list of blobs.
        myBlobs = myColorBlobLocatorProcessor.getBlobs();
        // The list of Blobs can be filtered to remove unwanted Blobs.
        //   Note:  All contours will be still displayed on the Stream Preview, but only those that satisfy the filter
        //             conditions will remain in the current list of "blobs".  Multiple filters may be used.
        //
        // Use any of the following filters.
        //
        // A Blob's area is the number of pixels contained within the Contour.  Filter out any that are too big or small.
        // Start with a large range and then refine the range based on the likely size of the desired object in the viewfinder.
        ColorBlobLocatorProcessor.Util.filterByArea(5000, 15000, myBlobs);
        // A blob's density is an indication of how "full" the contour is.
        // If you put a rubber band around the contour you would get the "Convex Hull" of the contour.
        // The density is the ratio of Contour-area to Convex Hull-area.
        ColorBlobLocatorProcessor.Util.filterByDensity(0.7, 1, myBlobs);
        // A blob's Aspect ratio is the ratio of boxFit long side to short side.
        // A perfect Square has an aspect ratio of 1.  All others are > 1
        // The list of Blobs can be sorted using the same Blob attributes as listed above.
        // No more than one sort call should be made.  Sorting can use ascending or descending order.
    }

    /**
     * Describe this function...
     */
    private void DisplayResults() {
        ColorBlobLocatorProcessor.Blob myBlob;
        RotatedRect myBoxFit;

        telemetry.addLine(" Area Density Aspect  Center");
        // Display the size (area) and center location for each Blob.
        for (ColorBlobLocatorProcessor.Blob myBlob_item : myBlobs) {
            myBlob = myBlob_item;
            // Get a "best-fit" bounding box (called "boxFit", of type RotatedRect) for this blob.
            myBoxFit = myBlob.getBoxFit();
            BlobDetected = true;
            PositionLR = Double.parseDouble(JavaUtil.formatNumber(myBoxFit.center.x, 1)) - 65;
            // Get the aspect ratio of this blob, i.e. the ratio of the
            // longer side of the "boxFit" bounding box to the shorter side.
            telemetry.addLine(JavaUtil.formatNumber(myBlob.getContourArea(), 5, 0) + "  " + JavaUtil.formatNumber(myBlob.getDensity(), 4, 2) + "   " + JavaUtil.formatNumber(myBlob.getAspectRatio(), 5, 2) + "  (" + JavaUtil.formatNumber(myBoxFit.center.x, 3, 0) + "," + JavaUtil.formatNumber(myBoxFit.center.y, 3, 0) + ")");
        }
    }
}
package org.firstinspires.ftc.teamcode.year2016_2017;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import for_camera_opmodes.OpModeCamera;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */

@Autonomous(name = "DetectColorSides", group = "ZZOpModeCameraPackage")
@Disabled
public class DetectColorSides extends OpModeCamera {

    int ds2 = 2;  // additional downsampling of the image
    private int looped = 0;
    private long lastLoopTime = 0;
// set to 1 to disable further downsampling

    /*
    * Code to run when the op mode is first enabled goes here
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
    */
    @Override
    public void init() {
        setCameraDownsampling(8);
// parameter determines how downsampled you want your images
// 8, 4, 2, or 1.
// higher number is more downsampled, so less resolution but faster
// 1 is original resolution, which is detailed but slow
// must be called before super.init sets up the camera

        super.init(); // inits camera functions, starts preview callback
    }

    public String colorName(int colorID) {
        switch (colorID) {
            case 0:
                return "RED";
            case 1:
                return "GREEN";
            case 2:
                return "BLUE";
            default:
                return "FECAL BROWN";
        }
    }

    /*
    * This method will be called repeatedly in a loop
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
    */
    @Override
    public void loop() {
        long startTime = System.currentTimeMillis();

        if (imageReady()) { // only do this if an image has been returned from the camera
            int redLeftValue = 0;
            int blueLeftValue = 0;
            int greenLeftValue = 0;
            int redRightValue = 0;
            int blueRightValue = 0;
            int greenRightValue = 0;

// get image, rotated so (0,0) is in the bottom left of the preview window
            Bitmap rgbImage;
            rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);

            for (int x = (rgbImage.getWidth() * 1 / 6); x < (rgbImage.getWidth() * 2 / 6); x++) {
                for (int y = (rgbImage.getHeight() * 1 / 4); y < (rgbImage.getHeight() * 3 / 4); y++) {
                    int pixel = rgbImage.getPixel(x, y);
                    redLeftValue += red(pixel);
                    blueLeftValue += blue(pixel);
                    greenLeftValue += green(pixel);
                }
            }
            for (int x = (rgbImage.getWidth() * 4 / 6); x < (rgbImage.getWidth() * 5 / 6); x++) {
                for (int y = (rgbImage.getHeight() * 1 / 4); y < (rgbImage.getHeight() * 3 / 4); y++) {
                    int pixel = rgbImage.getPixel(x, y);
                    redRightValue += red(pixel);
                    blueRightValue += blue(pixel);
                    greenRightValue += green(pixel);
                }
            }
            int colorLeft = highestColor(redLeftValue, greenLeftValue, blueLeftValue);
            int colorLeft2 = secondHighestColor(redLeftValue, greenLeftValue, blueLeftValue);
            int colorRight = highestColor(redRightValue, greenRightValue, blueRightValue);
            int colorRight2 = secondHighestColor(redRightValue, greenRightValue, blueRightValue);

            telemetry.addData("Left Highest Color:", colorName(colorLeft));
            telemetry.addData("Left 2nd Highest Color:", colorName(colorLeft2));
            telemetry.addData("Right Highest Color:", colorName(colorRight));
            telemetry.addData("Right 2nd Highest Color:", colorName(colorRight2));

        }
        long endTime = System.currentTimeMillis();
        telemetry.addData("Dims", Integer.toString(width / ds2) + " x " + Integer.toString(height / ds2));
        telemetry.addData("Loop Time", Long.toString(endTime - startTime));
        telemetry.addData("Loop to Loop Time", Long.toString(endTime - lastLoopTime));

        lastLoopTime = endTime;
    }

    @Override
    public void stop() {
        super.stop(); // stops camera functions
    }
}
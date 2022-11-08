
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Parameters;

import java.util.ArrayList;
import java.util.concurrent.BlockingQueue;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;



public class Vision {

    private LinearOpMode opMode;

    private VuforiaLocalizer vuforia;

    private final int RED_THRESHOLD = 100;
    private final int GREEN_MIN = 50;
    private final int GREEN_MAX = 230;
    private final int BLUE_THRESHOLD = 50;

    public Vision(LinearOpMode opMode) {
        this.opMode = opMode;

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AbrprCH/////AAABmedpMeggS0OPq0iPrr4kYlsH+h0grGS8/v1j2ucJLRDbqD6tRTW7y+f0Pw6/7QDlyYcEdrP/qwXnGeXbIykM5lmh8VjGt4x7e2+nDot9GrgQA1Fc2iYhLziqjWp3MePEJMcfmPHrEynXjI3rIy7QiYGsY1tiHS00OIUMo+Bh5HBHhuh0KCv9Yexa7P+28qBmaLu/tofXEKDvnNl61U/HrPz3s8IlzUAInMgRwA7VJLV42g8rG/hKfZJl/z83XCyq9mM95xTj2CrtHyKKQOOQIZY6QMBTT6l3dg5+JdT+diq0hHejoL8tHvajEfnTrBGnGhRPg8L1QBAW7x6ZmJqUvUtDTahryAc5+kR6e5RIPf3u";
        parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(4);
    }
    //gets picture of camera in vision
    public Bitmap getBitmap() throws InterruptedException{
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        Image rgb = frame.getImage(1);

        long numImages = frame.getNumImages();

        for (int i = 0; i < numImages; i++) {
            int fmt = frame.getImage(i).getFormat();

            if (fmt == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;

            }
            else {
                opMode.telemetry.addLine("Didn't find correct rgb format");
                opMode.telemetry.update();

            }

        }

        //gets pixels of the bitmap
        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        frame.close();

        opMode.telemetry.addLine("Got Bitmap");
        opMode.telemetry.update();

        opMode.sleep(500);

        return bm;
    }

    //the sample method is used to find the location of the block
    public String sample() throws InterruptedException{
        Bitmap bitmap = getBitmap();
        ArrayList<Integer> xValues = new ArrayList<>();

        opMode.telemetry.addData("height", bitmap.getHeight());

        int avgX = 0;

        //top left = (0,0
        //loops through each pixel, gets the rgb values
        for (int rowNum = 0; rowNum < bitmap.getWidth(); rowNum ++) {

            for (int colNum = 0; colNum < bitmap.getHeight(); colNum ++) {
                int pixel = bitmap.getPixel(rowNum, colNum);

                int greenPixel = green(pixel);
                int bluePixel = blue(pixel);
                int redPixel = red(pixel);

                if ((redPixel >= RED_THRESHOLD) && (greenPixel >= GREEN_MIN) && (greenPixel <= GREEN_MAX) && (bluePixel <= BLUE_THRESHOLD)) {
                    //if the pixel is colored within the color range we set, save its y value
                    xValues.add(rowNum);

                }

            }

        }

        //average all of the y pixels to find the average y position of the yellow color
        for (int x : xValues) {
            avgX+= x;

        }

        opMode.telemetry.addData("Num Pixels found", xValues.size());
        opMode.telemetry.update();

        try {
            avgX /= xValues.size();
        } catch (ArithmeticException E){ //catches divide by zero error
            return  "posA";

        }

        opMode.telemetry.addData("avgX = ", avgX);
        opMode.telemetry.update();
        opMode.sleep(1000);

        //checks the average y position and different positions correspond to different set ups

        if (xValues.size() == 0) {
            return "posC";

        }
        else if (avgX < 300) {
            return "posA";

        }
        else {
            return "posB";

        }

    }

}
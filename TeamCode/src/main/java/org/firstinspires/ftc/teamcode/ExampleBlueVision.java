package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by guinea on 10/5/17.
 * -------------------------------------------------------------------------------------
 * Copyright (c) 2018 FTC Team 5484 Enderbots
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * 
 * By downloading, copying, installing or using the software you agree to this license.
 * If you do not agree to this license, do not download, install,
 * copy or use the software.
 * -------------------------------------------------------------------------------------
 * A nice demo class for using OpenCVPipeline. This one also demonstrates how to use OpenCV to threshold
 * for a certain color (blue) and find contours of objects of that color, which is very common in
 * robotics OpenCV applications.
 */

public class ExampleBlueVision extends OpenCVPipeline {
    private boolean showContours = true;
    // To keep it such that we don't have to instantiate a new Mat every call to processFrame,
    // we declare the Mats up here and reuse them. This is easier on the garbage collector.
    private Mat hsv = new Mat();
    private Mat thresholded = new Mat();

    // this is just here so we can expose it later thru getContours.
    private List<MatOfPoint> contours = new ArrayList<>();

    public synchronized void setShowCountours(boolean enabled) {
        showContours = enabled;
    }

    private final Scalar GOLD = new Scalar(227,123,25);

    public  List<MatOfPoint> getContours() {

        return contours;
    }


    public double scalarDistance(Scalar s1, Scalar s2){
        return Math.sqrt(Math.pow(s1.val[0]-s2.val[0],2)+
                Math.pow(s1.val[1]-s2.val[1],2)+
                Math.pow(s1.val[2]-s2.val[2],2)+
                Math.pow(s1.val[3]-s2.val[3],2));
        //Pythagorean distance
    }


    // This is called every camera frame.
    @Override
    public Mat processFrame(Mat rgba, Mat gray) {


        // First, we change the colorspace from RGBA to HSV, which is usually better for color
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);
        // Then, we threshold our hsv image so that we get a black/white binary image where white
        // is the blues listed in the specified range of values
        // you can use a program like WPILib GRIP to find these values, or just play around.
        Core.inRange(hsv, new Scalar(90, 128, 30), new Scalar(170, 255, 255), thresholded);

        // we blur the thresholded image to remove noise
        // there are other types of blur like box blur or gaussian which can be explored.
        Imgproc.blur(thresholded, thresholded, new Size(3, 3));

        // create a list to hold our contours.
        // Conceptually, there is going to be a single contour for the outline of every blue object
        // that we can find. We can iterate over them to find objects of interest.
        // the Imgproc module has many functions to analyze individual contours by their area, avg position, etc.
        List<MatOfPoint> contours = new ArrayList<>();
        // this function fills our contours variable with the outlines of blue objects we found

        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);


        Log.d("glorp", contours.size()+ "");



   /*   if(contours.size() > 1 ){  Log.d("fleep", contours.get(0).toArray()[0] +"");}
       try{ if(contours.size() > 1 ){
            Point p1, p2;
            for (int i = 0; i < contours.size() - 1; i++ ){
                p1 = contours.get(i).toArray()[i];
                p2 = contours.get(i+1).toArray()[i+1];
                Imgproc.line (
                        rgba,                    //Matrix obj of the image
                        p1,        //p1
                        p2,       //p2
                        new Scalar(0, 0, 0),     //Scalar object for color
                        1                         //Thickness of the line
                );
            }}}
            catch(Exception e){
      e.printStackTrace();
      }*/

        // Then we display our nice little binary threshold on screen
        if (showContours) {
            // this draws the outlines of the blue contours over our original image.
            // they are highlighted in green.
            Imgproc.drawContours(rgba, contours, -1, new Scalar(0, 255, 0), 2, 8);
        }
        /*Imgproc.line (
                rgba,                    //Matrix obj of the image
                new Point(10, 200),        //p1
                new Point(300, 200),       //p2
                new Scalar(0, 0, 255),     //Scalar object for color
                5                          //Thickness of the line
        );*/

        return rgba; // display the image seen by the camera
    }
}

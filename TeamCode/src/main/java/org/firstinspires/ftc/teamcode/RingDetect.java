/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 *  all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/*
 *   Looks for yellow rectangles, and reports their aspect ratio. It will be
 * used in Ultimate Goal to count Rings in the Starter Stack.
 * o A case: no rectangles means no Rings. Put Wobble Goal in Zone A.
 * o B case: rectangle with short squat aspect ratio, 4 - 8. Go for Zone B.
 * o C case: taller rectangle, ratio 1-2. Go for Zone C.
 */

@TeleOp(name = "Ring Detect", group = "Vision test")
//@Disabled
public class RingDetect extends LinearOpMode {

  Pullbot robot = new Pullbot(this);
  //OpenCvInternalCamera2 phoneCam = new
  //OpenCvInternalCamera2 phoneCam;
  //Pullbot.RingOrientationAnalysisPipeline pipeline;

  @Override
  public void runOpMode() {
    /*
     * NOTE: Many comments have been omitted from EasyOpenCV's samples for
     * conciseness. If you're just starting out with EasyOpenCv,
     * you should take a look at {@link InternalCamera2Example} or its
     * webcam counterpart, {@link WebcamExample} first.
     */

    int ringsDetected = 0;
    // Create camera instance
    int cameraMonitorViewId =
        hardwareMap.appContext.getResources().getIdentifier(
            "cameraMonitorViewId", "id",
            hardwareMap.appContext.getPackageName());
    robot.phoneCam =
        OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

    // Open async and start streaming inside opened callback
    robot.phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
      @Override
      public void onOpened() {
        robot.phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);

        robot.pipeline = new Pullbot.RingOrientationAnalysisPipeline();
        robot.phoneCam.setPipeline(robot.pipeline);
      }
    });

    // Tell telemetry to update faster than the default 250ms period :)
    telemetry.setMsTransmissionInterval(20);

    waitForStart();

    while (opModeIsActive()) {
      // Don't burn an insane amount of CPU cycles in this sample because
      // we're not doing anything else
      sleep(20);

      // Report detected rectangles to telemetry. There should be only 1.
      // Todo: check this assertion.
      ArrayList<Pullbot.RingOrientationAnalysisPipeline.AnalyzedRing> rings =
          robot.pipeline.getDetectedRings();
      if (rings.isEmpty()) {
        // ringsDetected will be left at zero.
        telemetry.addLine("No rings detected.");
      } else {
        for (Pullbot.RingOrientationAnalysisPipeline.AnalyzedRing ring :
            rings) {
          // Ring discriminators. Throw out any Ring Stack too far to right,
          // too high, too wide or too tall.
          if (ring.left > 100) continue;
          if (ring.top < 140) continue;
          if (ring.width > 100) continue;
          if (ring.height > 60 ) continue;
          telemetry.addLine(String.format(Locale.US,
              "Ring aspect ratio = %.2f.",
              ring.aspectRatio));
          telemetry.addLine(String.format(Locale.US,
              "Ring top = %2d  left = %2d.",
              ring.top, ring.left));
          telemetry.addLine(String.format(Locale.US,
              "Ring width = %2d  height=%2d.",
              ring.width, ring.height));
          if (ring.aspectRatio > 1 && ring.aspectRatio <= 2) ringsDetected = 4;
          if (ring.aspectRatio > 2 && ring.aspectRatio <= 4) ringsDetected = 1;
          telemetry.addLine(String.format(Locale.US,
              "Rings detected = %2d.",
              ringsDetected));
        }
      }

      telemetry.update();
    }
  }
}
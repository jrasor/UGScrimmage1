package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * This is NOT an opmode
 *
 * This is an abstract Generic FTC robot class. From observation, robots do
 * have timers to count how long an opmode has been running. Not all robots
 * use vision.
 *
 * Version history
 * ======= =======
 * v 0.1    11/02/18 @Lorenzo Pedroza. Added this javadoc
 *          7/4/19 Coach Rasor added this to CSEE 331 code. Trainerbot and Lookeebot will inherit
 *          from this.
 * v 0.11   7/6/19 Coach Rasor moved some common opmode members into here.
 * v 0.12   9/25/20 An Ultimate Goal version.
 * v 0.13   10/24/20 In development: Vuforia assisted approach to Blue Tower
 *          Goal.
 */

public abstract class GenericFTCRobot {
  public GenericFTCRobot() {

  }
  private LinearOpMode currentOpMode;
  public GenericFTCRobot(LinearOpMode linearOpMode) {
    //super (linearOpMode);
    currentOpMode = linearOpMode;
  }

  public ElapsedTime runTime;
  public void init(){
    runTime = new ElapsedTime();
  }

  /*                      Robot independent measurements.                   */
  // Since ImageTarget trackables use mm to specify their dimensions, we must
  // use mm for all the physical dimensions.
  // We will define some constants and conversions here
  // TODO Get these as double without other objects griping.
  public static final float mmPerInch = 25.4f;
  // FTC Field is a square about 12' on a side.
  public static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;
  // Height of the center of the target images above the floor.
  public static final float mmTargetHeight = (6) * mmPerInch;

  public static final String VUFORIA_KEY =
      "ASkv3nr/////AAAAGYZ9CexhH0K0lDbV090F719DkwXCIXEUmExgnQNDFGjrDrk" +
          "VJnU7xNhuKHLsC32Pb1jmr+6vp6JtpVKvNmTf28ZYkUphDeajNPCLgGVxLjD6xs" +
          "fgBayqSO9bfQFeGkrdEgXlP+2oaz234afhWti9Jn8k71mzbQ4W2koX9yBMWz0YL" +
          "zUWClcasxi6Nty7SUvV+gaq3CzpKVtjKk+2EwV6ibIc0V47LAeB0lDGsGkSzuJ+" +
          "93/Ulpoj+Lwr/jbI2mu/Bs2W7U9mw73CMxvDix9o1FxyPNablla4W5C5lUDm0j2" +
          "lW5gsUNOhgvlWKQ+eCu9IBp53WbW5nfNzhXPaDDh/IlBbZuAMIJuMDEHI5PVLKT9L";
  /*                          Vision members.                         */
  /*
   * Get a Vuforia 'Development' license key for free from the Vuforia
   * developer
   * web site at https://developer.vuforia.com/license-manager.
   *
   * Once you have a license key, copy the string from the Vuforia web site
   * and paste it in to your code on the next line, between the double quotes.
   *
   * You can chop it up into substrings as shown here.
   */
  // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the
  // orientation, based on
  // how your phone is mounted:
  //      1) Camera Source.  Valid choices are:  BACK (behind screen) or
  //      FRONT (selfie side)
  //      2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true
  //      (portrait) or
  //      PHONE_IS_PORTRAIT = false (landscape)
  //
  // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam,
  // you must select
  // CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
  // A HardwarePullbot uses an Expansion Hub.
  //
  private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
  private static final boolean PHONE_IS_PORTRAIT = false;

  VuforiaLocalizer vuforia;

}
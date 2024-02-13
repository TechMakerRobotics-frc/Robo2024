package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;

 //import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * This is a demo program showing the use of OpenCV to do vision processing. The image is acquired
 * from the USB camera, then a rectangle is put on the image and sent to the dashboard. OpenCV has
 * many methods for different types of processing.
 */
public class LimelightSubsystem extends TimedRobot {
  Thread m_visionThread;

  @Override
  public void robotInit() {
    m_visionThread =
        new Thread(
            () -> {
              // Get the UsbCamera from CameraServer
              UsbCamera camera = CameraServer.startAutomaticCapture();
              // Set the resolution
              camera.setResolution(320, 180);

              // Get a CvSink. This will capture Mats from the camera
              CvSink cvSink = CameraServer.getVideo();
              // Setup a CvSource. This will send images back to the Dashboard
              CvSource outputStream = CameraServer.putVideo("SmartDashboard", 320, 180);

              // Mats are very memory expensive. Lets reuse this Mat.
              Mat mat = new Mat();

              // This cannot be 'true'. The program will never exit if it is. This
              // lets the robot stop this thread when restarting robot code or
              // deploying.
              while (!Thread.interrupted()) {
                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
                if (cvSink.grabFrame(mat) == 0) {
                  // Send the output the error.
                  outputStream.notifyError(cvSink.getError());
                  // skip the rest of the current iteration
                  continue;
                }
                // Put a rectangle on the image
                Imgproc.rectangle(
                    mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
                // Give the output stream a new image to display
                outputStream.putFrame(mat);
              }
            });
    m_visionThread.setDaemon(true);
    m_visionThread.start();


   /* private final NetworkTableEntry m_botPose;
    private final NetworkTableEntry m_camPose;
  
    public void Limelight() {
      m_botPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose");
      m_camPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace");
    }
  
  
      double[] botPose = m_botPose.getDoubleArray(new double[6]);
      double[] camPose = m_camPose.getDoubleArray(new double[6]);
     {

      if (botPose.length != 0) {
        SmartDashboard.putNumber("x bot pose", botPose[0]);
        SmartDashboard.putNumber("y bot pose", botPose[1]);
        SmartDashboard.putNumber("z bot pose", botPose[2]);
      }
  
      if (camPose.length != 0) {
        SmartDashboard.putNumber("x tag pose", camPose[0]);
        SmartDashboard.putNumber("y tag  pose", camPose[1]);
        SmartDashboard.putNumber("z tag pose", camPose[2]); */


    /* velForward = drivePID.calculate(limelight.getALimelight(), strafeOffSet);
        velStrafe = strafePID.calculate(limelight.getXLimelight(), strafeOffSet);
        velGiro = rotationPID.calculate(limelight.getYaw(),rotationOffset); */
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj.xrp.XRPOnBoardIO;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import org.photonvision.PhotonCamera;

// for teleop mode
import edu.wpi.first.wpilibj.XboxController;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // private final XRPDrivetrain m_drivetrain = new XRPDrivetrain();

    // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor leftMotor = new XRPMotor(0);
  private final XRPMotor rightMotor = new XRPMotor(1);
  private final DifferentialDrive mDrive = new DifferentialDrive(leftMotor, rightMotor);
  private final Timer mTimer = new Timer();
  private final XboxController mController = new XboxController(0);
  
  private final XRPOnBoardIO m_onboardIO = new XRPOnBoardIO();

  // Set up the XRPGyro
  private final XRPGyro m_gyro = new XRPGyro();

  // setup photon camera
  private final PhotonCamera m_camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
  private String state = "searching";

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    // SmartDashboard.putString("Example string", "Hello World");
    // SmartDashboard.putData("Auto choices", m_chooser);
    leftMotor.setInverted(true);
  }

  /**
   * Current angle of the XRP around the X-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the XRP around the Y-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the XRP around the Z-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }


  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    // m_drivetrain.resetEncoders();
    mTimer.start();
    mTimer.reset();
    SmartDashboard.putString("state", state);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // see if camera sees target
        var result = m_camera.getLatestResult();
        boolean target = result.hasTargets();
        double targetYaw;
        if (target){
           targetYaw = result.getBestTarget().getYaw();
        }else{
          targetYaw = 0;
        }

        SmartDashboard.putNumber("Target Yaw", targetYaw);
        SmartDashboard.putNumber("Gyro Angle", getGyroAngleZ());
        SmartDashboard.putBoolean("Target Found", target);
        state = "searching";

        if (target){
          // if it does, turn towards it
          // get target yaw angle
          // turn towards it
          double center_yaw = 0.0;
          double speed = 3.0 * (targetYaw - center_yaw)/40;
          // speed = Math.pow(speed, 2) * Math.signum(speed); // nonlinearity to avoid very small speeds
          double max_speed = 0.7;
          if(speed > max_speed){
            speed = max_speed;
          }
          SmartDashboard.putNumber("turn speed", speed);
          if (Math.abs(speed) > 0.1) {
            // larger than threshold, so turn
            mDrive.tankDrive(-speed, speed);
          } else {
            // dead band
            mDrive.tankDrive(0,0);
          }
          state = "turning";
          double target_area = result.getBestTarget().getArea();
          double yaw_threshold = 5.0;
          if (target_area < 1.0){
            yaw_threshold = 6;
          }
          if (target_area > 4){
            yaw_threshold = 4.0 + target_area;
          }
          SmartDashboard.putNumber("yaw threshold", yaw_threshold);
          boolean is_centered = Math.abs(targetYaw - center_yaw) < yaw_threshold;
          SmartDashboard.putBoolean("is_centered", is_centered);
          SmartDashboard.putNumber("Target Area", target_area);

          // estimate distance from target area
          double distance = 0.0;
          if (target_area > 0.0){
            distance = 1.0 / Math.sqrt(target_area);
          }
          SmartDashboard.putNumber("Distance", distance);
          double distance_min = 1/Math.sqrt(17.5);  // closest to get to target

          if (is_centered & distance > distance_min){
            // if centered and not close enough, move forward
            double forward_speed = 6.0 * (distance - distance_min);
            SmartDashboard.putNumber("forward speed", forward_speed);
            double max_forward_speed = 4.0;
            if (forward_speed > max_forward_speed){
              forward_speed = max_forward_speed;
            }
            if (forward_speed > 0.1) {
              // larger than threshold, so move forward
              mDrive.tankDrive(forward_speed, forward_speed);
              state = "approaching";
            } else {
              // dead band
              mDrive.tankDrive(0,0);
            }
          } else if (is_centered){
            // if centered and close to target then stop
            mDrive.tankDrive(0,0);
            state = "found";
          }
        } else {  // no target
          // if no target, turn in place slowly, looking for target
          double turn_speed = 0.8;
          double now = mTimer.get();
          double now_seconds = now % 0.8;
          // only turn if within 0.2 seconds of 2 second interval
          if (now_seconds > 0.2){
            turn_speed = 0;
          }
          mDrive.tankDrive(-turn_speed, turn_speed);
          state = "searching";
        }
        SmartDashboard.putString("state", state);
        break;
      }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // mDrive.tankDrive(mController.getLeftY(), mController.getLeftX());
    mDrive.arcadeDrive(-mController.getLeftY(), mController.getLeftX());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}

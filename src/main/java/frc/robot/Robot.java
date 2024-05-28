// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotState;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  AHRS ahrs = new AHRS(SPI.Port.kMXP);
  private static final String kDefaultAuto = "Default";
  private static final String kShootLeaveAuto = "Shoot leave";
  private static final String kShootLeaveShootAuto = "Shoot leave shoot";
  private static final String kShootLeaveOffCenterAuto = "Shoot leave off center";
  private static final String kShootLeaveShootOffCenterAuto = "Shoot leave shoot off center";
  private static final String kOffCenterJustShoot = "Off center just shoot";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private boolean useFull;
  private boolean runAuto;
  // defining drive here
  private final XboxController m_Drivestick = new XboxController(0);
  //private final XboxController m_Operatorstick = new XboxController(1);
  R2Jesu_Drive robotDrive = new R2Jesu_Drive(ahrs);
  R2Jesu_Limelight robotLimelight = new R2Jesu_Limelight();
  R2Jesu_Align robotAlign = new R2Jesu_Align(robotDrive, robotLimelight, ahrs);
  R2Jesu_Shooter robotShooter = new R2Jesu_Shooter(robotLimelight);
  R2Jesu_Hanger robotHanger = new R2Jesu_Hanger();
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Shoot leave", kShootLeaveAuto);
    m_chooser.addOption("Shoot leave shoot", kShootLeaveShootAuto);
    m_chooser.addOption("Shoot leave off center", kShootLeaveOffCenterAuto);
    m_chooser.addOption("Shoot leave shoot off center", kShootLeaveShootOffCenterAuto);
    m_chooser.addOption("Off Center Just Shoot", kOffCenterJustShoot);
    SmartDashboard.putData("Auto choices", m_chooser);
    SignalLogger.enableAutoLogging(false);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    robotLimelight.postDashboard();
    robotDrive.driveMetrics();
    robotLimelight.getDistance();
  }

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
    runAuto = true;
  }

  /** This function is called periodically during autonomous. */
  @Override
  // shoot leave fixed!!!!!!!!!!!!!!!
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kShootLeaveAuto:
        if (runAuto) {
          robotDrive.zeroPosition();
          robotDrive.drive(0.0, 0.0, 0.0, false);
          robotShooter.shoot();
          try {
            Thread.sleep(1000);
          }
          catch (InterruptedException e)
          {
            System.out.println("Sleep");
          }
          while (robotDrive.driveAuto(0.0, -0.30, 0.0, 0.0) < 60 && RobotState.isAutonomous()) {
              robotShooter.runShooter(m_Drivestick);
          }
          robotDrive.drive(0.0, 0.0, 0.0, false);
          runAuto = false;
        }
        break;
      case kShootLeaveShootAuto:
        if (runAuto) {
           System.out.println("Auto start");
          robotShooter.justShooter();
          robotDrive.zeroPosition();
          try {
            Thread.sleep(500);
          }
          catch (InterruptedException e)
          {
            System.out.println("Sleep");
          }
          //System.out.println("drive 72");
          robotShooter.shoot();
          try {
            Thread.sleep(1000);
          }
          catch (InterruptedException e)
          {
            System.out.println("Sleep");
          }
          while (robotDrive.driveAuto(0.0, -0.50, 0.0, 0.0) < 72 && RobotState.isAutonomous())
          {
            robotShooter.runShooter(m_Drivestick);
          };
           System.out.println("bye speaker");
          //robotDrive.drive(0.0, 0.0, 0.0, false);
          robotDrive.driveAuto(0.0, 0.0, 0.0, 0.0);
          robotDrive.zeroPosition();
          try {
            Thread.sleep(200);
          }
          catch (InterruptedException e)
          {
            System.out.println("Sleep");
          }
          //System.out.println("drive 72");
          while (robotDrive.driveAuto(0.0, 0.50, 0.0, 0.0) < 72 && RobotState.isAutonomous());
           System.out.println("hello speaker");
          //robotDrive.drive(0.0, 0.0, 0.0, false);
          robotDrive.driveAuto(0.0, 0.0, 0.0, 0.0);
          robotShooter.shoot();
          try {
            Thread.sleep(1000);
          }
          catch (InterruptedException e)
          {
            System.out.println("Sleep");
          }
          robotDrive.zeroPosition();
          //robotDrive.drive(0.0, 0.0, 0.0, false);
          runAuto = false;
           System.out.println("end auto");
        }
        break;
      case kShootLeaveOffCenterAuto:
        if (runAuto) {
          robotDrive.zeroPosition();
          robotShooter.justShooter();
          try {
            Thread.sleep(200);
          }
          catch (InterruptedException e)
          {
            System.out.println("Sleep");
          }
          //robotDrive.drive(0.0, 0.0, 0.0, false);
          robotShooter.shoot();
          try {
            Thread.sleep(1000);
          }
          catch (InterruptedException e)
          {
            System.out.println("Sleep");
          }
          while (robotDrive.driveAutoOffCenter(0.30, 0.0, 0.0) < 22 && RobotState.isAutonomous());
          robotDrive.drive(0.0, 0.0, 0.0, false);
          while (!(robotDrive.turnToAngle(0.0)) && RobotState.isAutonomous());
          robotDrive.zeroPosition();
          try {
            Thread.sleep(200);
          }
          catch (InterruptedException e)
          {
            System.out.println("Sleep");
          }
          while (robotDrive.driveAuto(0.0, -0.45, 0.0, 0.0) < 75 && RobotState.isAutonomous()) {
              robotShooter.runShooter(m_Drivestick);
          }
          robotDrive.drive(0.0, 0.0, 0.0, false);
          runAuto = false;
        }
        break;

        case kShootLeaveShootOffCenterAuto:
        if (runAuto) {
          robotDrive.zeroPosition();
          double autoAngle = robotDrive.getAngle();
          if (autoAngle < 0)
          {
            autoAngle += 10;
          }
          else
          {
            autoAngle -= 10;
          }
          robotShooter.justShooter();
          try {
            Thread.sleep(1000);
          }
          catch (InterruptedException e)
          {
            System.out.println("Sleep");
          }
          robotShooter.shoot();
          try {
            Thread.sleep(800);
          }
          catch (InterruptedException e)
          {
            System.out.println("Sleep");
          }

          while (robotDrive.driveAutoOffCenter(0.40, 0.0, 0.0) < 22 && RobotState.isAutonomous());
          robotDrive.drive(0.0, 0.0, 0.0, false);
          while (!(robotDrive.turnToAngle(0.0)) && RobotState.isAutonomous());
          robotDrive.zeroPosition();
          try {
            Thread.sleep(200);
          }
          catch (InterruptedException e)
          {
            System.out.println("Sleep");
          }
          //System.out.println("drive 65");
          while (robotDrive.driveAuto(0.0, -0.50, 0.0, 0.0) < 75 && RobotState.isAutonomous()) {
              robotShooter.runShooter(m_Drivestick);
          }
          robotDrive.drive(0.0, 0.0, 0.0, false);
          //robotShooter.runShooter(m_Drivestick);
          //robotDrive.drive(0.0, 0.0, 0.0, false);
          robotDrive.zeroPosition();
          try {
            Thread.sleep(200);
          }
          catch (InterruptedException e)
          {
            System.out.println("Sleep");
          }
          //System.out.println("drive 58");
          while (robotDrive.driveAuto(0.0, 0.50, 0.0, 0.0) < 72 && RobotState.isAutonomous());
          try {
            Thread.sleep(200);
          }
          catch (InterruptedException e)
          {
            System.out.println("Sleep");
          }
          //System.out.println("turn to angle 2");
          robotShooter.justShooter();
          while (!(robotDrive.turnToAngle(autoAngle)) && RobotState.isAutonomous());
          robotDrive.drive(0.0, 0.0, 0.0, false);
          robotDrive.zeroPosition();
          try {
            Thread.sleep(200);
          }
          catch (InterruptedException e)
          {
            System.out.println("Sleep");
          }
          while (robotDrive.driveAutoOffCenter(-0.40, 0.0, 0.0) < 22 && RobotState.isAutonomous());
          robotDrive.drive(0.0, 0.0, 0.0, false);
          //robotDrive.drive(0.0, 0.0, 0.0, false);
          robotShooter.shoot();
          runAuto = false;
        }
        break;
      case kOffCenterJustShoot:
      if (runAuto) {
        robotShooter.justShooter();
          try {
            Thread.sleep(1000);
          }
          catch (InterruptedException e)
          {
            System.out.println("Sleep");
          }
          robotShooter.shoot();
          robotDrive.drive(0.0, 0.0, 0.0, false);
          runAuto = false;
        }
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
      
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    robotDrive.drive(0, 0, 0, false);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Full?", m_Drivestick.getLeftTriggerAxis());
    if (m_Drivestick.getLeftTriggerAxis() <= 0.0) {
      useFull = false;
    }
    else {
      useFull = true;
    }

    robotDrive.drive(m_Drivestick.getRightX(), m_Drivestick.getRightY(), m_Drivestick.getLeftX(), this.useFull);
    robotShooter.runShooter(m_Drivestick);
    robotHanger.hang(m_Drivestick);
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

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

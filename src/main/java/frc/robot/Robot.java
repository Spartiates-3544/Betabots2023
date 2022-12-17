// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private WPI_TalonFX frontLeft = new WPI_TalonFX(4);
  private WPI_TalonFX frontRight = new WPI_TalonFX(2);
  private WPI_TalonFX backLeft = new WPI_TalonFX(1);
  private WPI_TalonFX backRight = new WPI_TalonFX(3);
  private WPI_TalonFX conveyor = new WPI_TalonFX(5);
  private XboxController controller = new XboxController(0);
  //private AHRS gyro = new AHRS(SerialPort.Port.kMXP);
  private AnalogGyro gyro = new AnalogGyro(1);
  private MecanumDrive drive = new MecanumDrive(frontLeft, backLeft, frontLeft, frontRight);
  private Timer timer = new Timer();
  private Timer timer2 = new Timer();
  private Timer timer3 = new Timer();
  private Timer timer4 = new Timer();
  private Timer timer5 = new Timer();
  private Timer timer6 = new Timer();
  private Timer timer7 = new Timer();
  private Timer timer8 = new Timer();


  private boolean fieldOriented = true;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    gyro.calibrate();
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
    frontRight.setInverted(InvertType.InvertMotorOutput);
    backRight.setInverted(InvertType.InvertMotorOutput);
    timer7.start();
  }

  @Override
  public void autonomousPeriodic() {
    //please oh god save me from this mess
    while (timer7.get() < 0.3) {
      drive.driveCartesian(0.2, 0, 0);
    }
    drive.driveCartesian(0.0, 0, 0);
    timer7.stop();
    timer8.start();
    while (timer8.get() < 1.55) {
      drive.driveCartesian(0, 0.15, 0);
    }
    drive.driveCartesian(0.0, 0, 0);
    timer8.stop();
    timer.start();
    while (timer.get() < 2) {
      drive.driveCartesian(-0.2, 0, 0);
    }
    drive.driveCartesian(0.0, 0, 0);
    timer.stop();
    timer2.start();
    while (timer2.get() < 0.2) {
      drive.driveCartesian(0, 0.15, 0);
    }
    drive.driveCartesian(0, 0, 0);
    timer2.stop();
    timer3.start();
    while (timer3.get() < 0.65) {
      drive.driveCartesian(-0.2, 0, 0);
    }
    drive.driveCartesian(0, 0, 0);
    timer3.stop();
    timer4.start();
    while (timer4.get() < 6) {
      conveyor.set(0.2);      
    }
    timer4.stop();
    conveyor.stopMotor();
    timer5.start();
    while (timer5.get() < 1) {
      drive.driveCartesian(0.2, 0, 0);
    }
    timer5.stop();
    drive.driveCartesian(0, 0, 0);
    timer6.start();
    while (timer6.get() < 0.8) {
      drive.driveCartesian(-0.2, 0, 0);
    }
  }
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    frontRight.setInverted(InvertType.InvertMotorOutput);
    backRight.setInverted(InvertType.InvertMotorOutput);
    drive.setDeadband(0.02);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putData(drive);
    SmartDashboard.putNumber("Angle", gyro.getAngle());
    SmartDashboard.putBoolean("Field oriented", fieldOriented);

    //Field oriented toggle
    if (controller.getBButtonPressed()) fieldOriented = !fieldOriented;

    if (fieldOriented) {
      //thomas drive
      //drive.driveCartesian(-(controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()) * .5, -controller.getRightX() * .3, -controller.getLeftX() * .5, gyro.getAngle());

      //other drive
      drive.driveCartesian(controller.getLeftY() * .3, -controller.getLeftX() * .3, -controller.getRightX() * .3, gyro.getAngle());
    }
    else if (!fieldOriented) {
      //thomas drive
      //drive.driveCartesian(-(controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()) * .5, -controller.getRightX() * .3, -controller.getLeftX() * .5);

      //other drive
      drive.driveCartesian(controller.getLeftY() * .3, -controller.getLeftX() * .3, -controller.getRightX() * .3);
    }


    if (controller.getRightBumper()) {
      conveyor.set(0.3);
      controller.setRumble(RumbleType.kLeftRumble, 0.2);
      controller.setRumble(RumbleType.kRightRumble, 0.2);
    } 
    else if (controller.getLeftBumper()) {
      conveyor.set(-0.3);
      controller.setRumble(RumbleType.kLeftRumble, 0);
      controller.setRumble(RumbleType.kRightRumble, 0);

    } else {
      conveyor.stopMotor();
      controller.setRumble(RumbleType.kLeftRumble, 0);
      controller.setRumble(RumbleType.kRightRumble, 0);
    }
    if (controller.getAButton()) gyro.calibrate();

    drive.feed();
  }
}
//constant integral.PIDrobot.jpg//
//mp3.megalovania;Falcon500FX.//


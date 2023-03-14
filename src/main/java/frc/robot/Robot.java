// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Import the libraries we need

// Using the TimedRobot paradigm
import edu.wpi.first.wpilibj.TimedRobot;
// Using the Mecanum omnidirectional Drive System
import edu.wpi.first.wpilibj.drive.MecanumDrive;
// Motor Controller used to control the lights
import edu.wpi.first.wpilibj.motorcontrol.Spark;
// Using Rev Robotics SparkMax motor controllers through
// the CAN interface
import com.revrobotics.CANSparkMax;
// Sparkmax/Neo encoders
import com.revrobotics.RelativeEncoder;
// Our motor type is "Brushless" using the Rev Neo
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// Pneumaticws libraries
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
// Xbox controllers for our joystics
import edu.wpi.first.wpilibj.XboxController;
// Camera Server for USB cameras
import edu.wpi.first.cameraserver.CameraServer;
// USB Cameras
import edu.wpi.first.cscore.UsbCamera;
// Driver station info for game data
import edu.wpi.first.wpilibj.DriverStation;
// Dashboard for autonomous selection
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Program the robot...
public class Robot extends TimedRobot 
{
  // CAN IDs of each motor on the robot 
  private static final int frontLeftCANID = 4;
  private static final int rearLeftCANID = 3;
  private static final int frontRightCANID = 6;
  private static final int rearRightCANID = 7;
  private static final int armElevationCANID = 5;
  private static final int armExtensionCANID = 8;
  private static final int clawExtendPWM = 0;
  private static final int clawRetractPWM = 1;

  // Drivetrain Motors
  private CANSparkMax frontLeftMotor = new CANSparkMax(frontLeftCANID, MotorType.kBrushless);
  private CANSparkMax frontRightMotor = new CANSparkMax(frontRightCANID, MotorType.kBrushless);
  private CANSparkMax rearLeftMotor = new CANSparkMax(rearLeftCANID, MotorType.kBrushless);
  private CANSparkMax rearRightMotor = new CANSparkMax(rearRightCANID, MotorType.kBrushless);

  // Drivetrain
  private MecanumDrive m_robotDrive;

  // Arm Motors
  private CANSparkMax armExtensionMotor = new CANSparkMax(armExtensionCANID, MotorType.kBrushless);
  private CANSparkMax armElevationMotor = new CANSparkMax(armElevationCANID, MotorType.kBrushless);
  private RelativeEncoder armElevationEncoder;

  // Claw Solenoid
  public DoubleSolenoid clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, clawExtendPWM, clawRetractPWM); 

  // Xbox controller for driver
  private XboxController xbox_drive;

  // Xbox controller for operator
  private XboxController xbox_operator;

  // Top Camera
  UsbCamera topcam;

  // Bottom Camera
  UsbCamera bottomcam;

  // LED Lights Controller
  private final Spark lights = new Spark(1);

  // AUTONOMOUS CODE
  @Override
  public void autonomousInit() 
  {
    // reset all encoders
  }

  @Override
  public void autonomousPeriodic()
  {
    // Autonomous routine template
    // -- Back onto charge station left
    // -- back onto charge station right
    // -- back onto charge station center
    // -- autobalance?
    // -- just touch ramp to dock?
    // -- go straight back
    // -- back around charge station left
    // -- back around charge station right
    // -- spin to face game pieces?
    //
    // lower arm to set position
    // extend arm
    // open claw
    // retract arm
    // roll to correct scenario
  }

  // TELEOP CODE
  @Override
  public void robotInit() 
  {
    frontLeftMotor = new CANSparkMax(frontLeftCANID, MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(frontRightCANID, MotorType.kBrushless);
    rearLeftMotor = new CANSparkMax(rearLeftCANID, MotorType.kBrushless);
    rearRightMotor = new CANSparkMax(rearRightCANID, MotorType.kBrushless);

    // Invert motors that spin the wrong way during testing
    frontRightMotor.setInverted(true);
    rearRightMotor.setInverted(true);

    // Limit our arm motors for safety
    armExtensionMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    armExtensionMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    armExtensionMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 168);
    armExtensionMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 2);
    armElevationMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    armElevationMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    armElevationMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 50);
    armElevationMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 11);
    armElevationEncoder = armElevationMotor.getEncoder();

    // Robot drivetrain subsystem
    m_robotDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

    // XBox Controllers
    xbox_drive = new XboxController(0);
    xbox_operator = new XboxController(1);

    // Start drive camera
    topcam = CameraServer.startAutomaticCapture(0);

    // Start floor camera
    bottomcam = CameraServer.startAutomaticCapture(1);

    // Turn on the lights
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue)
    {
      // Breath, Blue
      lights.set(-0.15); 
    }
    else if (DriverStation.getAlliance() == DriverStation.Alliance.Red)
    {
      // Breath, Red
      lights.set(-0.17);
    }
    else
    {
      // Color Waves, Rainbow Palette
      lights.set(-0.45);
    }
  }

  @Override
  public void teleopPeriodic() 
  {
    // The section executes repeatedly when the robot enters TeleOp mode

    // Left joystick controls driving and strafing in any direction.
    // Right joystick controls spinning in place
    m_robotDrive.driveCartesian(-1*xbox_drive.getLeftY(), xbox_drive.getLeftX(), xbox_drive.getRightX());
    
    // Extend Arm
    if (xbox_operator.getBButton())
    {
      System.out.println("B Button - Extend Arm!");
      armExtensionMotor.set(.1);
    }
    // Retract arm
    else if (xbox_operator.getAButton())
    {
      System.out.println("A Button - Retract Arm!");
      armExtensionMotor.set(-.1);
    } 
    // Lower arm
    else if (xbox_operator.getXButton())
    {
      System.out.println("X Button - Lower Arm!");
      armElevationMotor.set(.1);
    } 
    // Raise arm
    else if (xbox_operator.getYButton())
    {
      System.out.println("Y Button - Raise Arm!");
      armElevationMotor.set(-.1);
    } 
    // Open Claw
    else if (xbox_operator.getLeftBumper())
    {
      System.out.println("Left Bumper - Open Claw!");
      clawSolenoid.set(kReverse);
    } 
    // Close Claw
    else if (xbox_operator.getRightBumper())
    {
      System.out.println("Right Bumper - Close Claw!");
      clawSolenoid.set(kForward);
    } 
  }
}

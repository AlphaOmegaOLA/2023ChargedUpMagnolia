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
// Sparkmax PID Controller to smooth out arm movement
import com.revrobotics.SparkMaxPIDController;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Program the robot...
public class Robot extends TimedRobot 
{
  // CAN IDs of each motor on the robot 
  private static final int leftFrontCANID = 4;
  private static final int leftRearCANID = 3;
  private static final int rightFrontCANID = 6;
  private static final int rightRearCANID = 7;
  private static final int armPivotCANID = 5;
  private static final int armExtensionCANID = 8;
  private static final int clawExtendPWM = 0;
  private static final int clawRetractPWM = 1;

  // Drivetrain Motors
  private CANSparkMax leftFrontMotor = new CANSparkMax(leftFrontCANID, MotorType.kBrushless);
  private CANSparkMax rightFrontMotor = new CANSparkMax(rightFrontCANID, MotorType.kBrushless);
  private CANSparkMax leftRearMotor = new CANSparkMax(leftRearCANID, MotorType.kBrushless);
  private CANSparkMax rightRearMotor = new CANSparkMax(rightRearCANID, MotorType.kBrushless);

  // Drivetrain
  private MecanumDrive driveTrain;

  // Arm Motors
  private CANSparkMax armExtensionMotor = new CANSparkMax(armExtensionCANID, MotorType.kBrushless);
  private CANSparkMax armPivotMotor = new CANSparkMax(armPivotCANID, MotorType.kBrushless);
  private RelativeEncoder armPivotEncoder;
  private SparkMaxPIDController armPivotPIDController;

  // PID coefficients
  public double kP = 0.2; 
  public double kI = 0.0001;
  public double kD = .0008; 
  public double kIz = 30; 
  public double kFF = 0; 
  public double kMaxOutput = .3; 
  public double kMinOutput = -.3;
  public int armDownRotations = 45;
  public int armUpRotations = 2;

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
    // Invert motors that spin the wrong way during testing
    rightFrontMotor.setInverted(true);
    rightRearMotor.setInverted(true);

    // Limit our arm motors for safety
    armExtensionMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    armExtensionMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    armExtensionMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 168);
    armExtensionMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 2);
    armPivotMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    armPivotMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    armPivotMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 48);
    armPivotMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 1);
    armPivotEncoder = armPivotMotor.getEncoder();
    armPivotPIDController = armPivotMotor.getPIDController();

    // Robot drivetrain subsystem
    driveTrain = new MecanumDrive(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);

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
    driveTrain.driveCartesian(-1*xbox_drive.getLeftY(), xbox_drive.getLeftX(), xbox_drive.getRightX()*.5);
    
    // Display the arm Pivot encoder position on the dashboard (rotations)
    SmartDashboard.putNumber("Arm Pivot Encoder Position", armPivotEncoder.getPosition());
    SmartDashboard.putNumber("Arm Pivot Encoder Velocity", armPivotEncoder.getVelocity());

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
    else if (xbox_operator.getYButton())
    {
      System.out.println("Y Button - Lower Arm!");
      //armPivotMotor.set(.1);
      armPivotPIDController.setP(kP);
      armPivotPIDController.setI(kI);
      armPivotPIDController.setD(kD);
      armPivotPIDController.setIZone(kIz);
      armPivotPIDController.setFF(kFF);
      armPivotPIDController.setOutputRange(kMinOutput, kMaxOutput);
      armPivotPIDController.setReference(armDownRotations, CANSparkMax.ControlType.kPosition);
    } 
    // Raise arm
    else if (xbox_operator.getXButton())
    {
      System.out.println("X Button - Raise Arm!");
      armPivotPIDController.setP(kP);
      armPivotPIDController.setI(kI);
      armPivotPIDController.setD(kD);
      armPivotPIDController.setIZone(kIz);
      armPivotPIDController.setFF(kFF);
      armPivotPIDController.setOutputRange(kMinOutput, kMaxOutput);
      armPivotPIDController.setReference(armUpRotations, CANSparkMax.ControlType.kPosition);
    } 
    // Open Claw
    else if (xbox_operator.getLeftBumper())
    {
      System.out.println("Left Bumper - Close Claw!");
      clawSolenoid.set(kForward);
    } 
    // Close Claw
    else if (xbox_operator.getRightBumper())
    {
      System.out.println("Right Bumper - Open Claw!");
      clawSolenoid.set(kReverse);
    } 
  }
}

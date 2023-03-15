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

  // Drivetrain speeds - pressing right trigger slows it down
  // Fast driving speed
  private double driveFast = 1.0;
  // Slow driving speed
  private double driveSlow = .3;
  // Fast rotation
  private double rotateFast = .5;
  // Slow rotation
  private double rotateSlow = .2;
  // Scaling factor to slow down drive speed
  private double driveScale = driveFast;
  // Scaling factor to slow down rotation speed
  private double rotateScale = rotateFast;

  // Arm Motors
  private CANSparkMax armExtensionMotor = new CANSparkMax(armExtensionCANID, MotorType.kBrushless);
  private CANSparkMax armPivotMotor = new CANSparkMax(armPivotCANID, MotorType.kBrushless);
  private RelativeEncoder armPivotEncoder;
  private SparkMaxPIDController armPivotPIDController;

  /* PID CONTROLLER COEFFICIENTS
     The PID controller is what makes our arm move
     as smoothly as possible with as little bounce
     as possible. 
  */ 
  // (P)roportional Value - Starting power (20% of motor capacity)
  public double kP = 0.2; 
  // (I)ntegral Value - Detects if the motor is struggling by
  // summing up the error (remaining distance) and "nudging"
  // the power until the error value decreases
  public double kI = 0.0001;
  // (D)ifferential Value - slows the motor down as the
  // motor gets closer to its goal 
  public double kD = .0008; 
  // I Zone - limits when the I value starts keeping track
  public double kIz = 30; 
  // Feed forward is the minimum amount of power we think
  // we need so the motor doesn't struggle ramping up. Our
  // arm is balanced enough that we don't seem to need this.
  public double kFF = 0; 
  // The min and max output limit how much the motor can surge
  public double kMaxOutput = .3; 
  public double kMinOutput = -.3;
  // These are the rotations we want for each arm position
  //
  // Pick things up off the floor
  public int armFloorRotations = 45;
  // Grab an upright cone at the top
  public int armConeTopRotations = 40;
  // Go to scoring angle
  public int armScoreRotations = 2;
  // Raise arm high for travelS
  public int armTravelRotations = 1;

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

    // Extend arm
    armExtensionMotor.set(.3);

    // Drop game piece
    clawSolenoid.set(kReverse);

    // Retract Arm
    armExtensionMotor.set(-.3);

    // Now we need to check which scenario we're in


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

    // Slow down robot for precision driving
    if (xbox_drive.getRightTriggerAxis() > 0)
    {
      driveScale = driveSlow;
      rotateScale = rotateSlow;
      System.out.println("Driving SLOW....");
    }
    else
    {
      driveScale = driveFast;
      rotateScale = rotateFast;
      System.out.println("Driving FAST....");
    }

    // Left joystick controls driving and strafing in any direction.
    // Right joystick controls spinning in place
    driveTrain.driveCartesian(-1*xbox_drive.getLeftY()*driveScale, xbox_drive.getLeftX()*driveScale, xbox_drive.getRightX()*rotateScale);
    
    // Display the arm Pivot encoder position on the dashboard (rotations)
    SmartDashboard.putNumber("Arm Pivot Encoder Position", armPivotEncoder.getPosition());
    SmartDashboard.putNumber("Arm Pivot Encoder Velocity", armPivotEncoder.getVelocity());

    // Raise arm all the way to travel
    if (xbox_operator.getAButton())
    {
      System.out.println("A Button - Raise Arm to travel!");
      armPivotPIDController.setP(kP);
      armPivotPIDController.setI(kI);
      armPivotPIDController.setD(kD);
      armPivotPIDController.setIZone(kIz);
      armPivotPIDController.setFF(kFF);
      armPivotPIDController.setOutputRange(kMinOutput, kMaxOutput);
      armPivotPIDController.setReference(armTravelRotations, CANSparkMax.ControlType.kPosition);
    }
    // Move arm into scoring position
    else if (xbox_operator.getBButton())
    {
      System.out.println("B Button - Move arm to score!");
      armPivotPIDController.setP(kP);
      armPivotPIDController.setI(kI);
      armPivotPIDController.setD(kD);
      armPivotPIDController.setIZone(kIz);
      armPivotPIDController.setFF(kFF);
      armPivotPIDController.setOutputRange(kMinOutput, kMaxOutput);
      armPivotPIDController.setReference(armScoreRotations, CANSparkMax.ControlType.kPosition);
    } 
    // Move arm to the top of an upright cone
    else if (xbox_operator.getXButton())
    {
      System.out.println("X Button - Move arm to top of an upright cone!");
      armPivotPIDController.setP(kP);
      armPivotPIDController.setI(kI);
      armPivotPIDController.setD(kD);
      armPivotPIDController.setIZone(kIz);
      armPivotPIDController.setFF(kFF);
      armPivotPIDController.setOutputRange(kMinOutput, kMaxOutput);
      armPivotPIDController.setReference(armConeTopRotations, CANSparkMax.ControlType.kPosition);
    } 
    // Lower arm to floor to pick up game pieces
    else if (xbox_operator.getYButton())
    {
      System.out.println("Y Button - Lower arm to floor!");
      armPivotPIDController.setP(kP);
      armPivotPIDController.setI(kI);
      armPivotPIDController.setD(kD);
      armPivotPIDController.setIZone(kIz);
      armPivotPIDController.setFF(kFF);
      armPivotPIDController.setOutputRange(kMinOutput, kMaxOutput);
      armPivotPIDController.setReference(armFloorRotations, CANSparkMax.ControlType.kPosition);
    } 
    // Retract Arm
    else if (xbox_operator.getLeftBumper())
    {
      armExtensionMotor.set(-.3);
      System.out.println("Left Bumper - Retract Arm!");
    }
    // Extend Arm
    else if (xbox_operator.getRightBumper())
    {
      armExtensionMotor.set(.3);
      System.out.println("Right Bumper - Extend Arm!");
    }
    // Open Claw
    else if (xbox_operator.getLeftTriggerAxis() > 0)
    {
      System.out.println("Left Trigger - Close Claw!");
      clawSolenoid.set(kForward);
    } 
    // Close Claw
    else if (xbox_operator.getRightTriggerAxis() > 0)
    {
      System.out.println("Right Trigger - Open Claw!");
      clawSolenoid.set(kReverse);
    } 
  }
}

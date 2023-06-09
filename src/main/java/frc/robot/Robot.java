// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Import the libraries we need

// Countdown Timers
import edu.wpi.first.wpilibj.Timer;
// Using the TimedRobot paradigm
import edu.wpi.first.wpilibj.TimedRobot;
// Using the Mecanum omnidirectional Drive System
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// Motor Controller used to control the lights
import edu.wpi.first.wpilibj.motorcontrol.Spark;
// Using Rev Robotics SparkMax motor controllers through
// the CAN interface for arm and PWM for everything else
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.CANSparkMax;
// Sparkmax/Neo encoders
import com.revrobotics.RelativeEncoder;
// Sparkmax PID Controller to smooth out arm movement
import com.revrobotics.SparkMaxPIDController;
// Our motor type is "Brushless" using the Rev Neo
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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
// Data Logger
import edu.wpi.first.wpilibj.DataLogManager;
// Limit joystick jumpiness
import edu.wpi.first.math.filter.SlewRateLimiter;
// Autonomous menu selection
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

// Program the robot...
public class Robot extends TimedRobot 
{
  // Timer for autonomous
  Timer autoTimer = new Timer();

  // For driverstation match information
  private boolean hasMatchName = false;

  // Slewrate limiter to dampen joystick rate change
  SlewRateLimiter filter = new SlewRateLimiter(0.2);

  // AutoBalance engine
  // AutoBalance code uses the RoboRio 2 built-in IMU.
  // Code taken from GitHub link in this post: https://www.chiefdelphi.com/t/psa-balance-in-auto/429778
  private AutoBalance autoBalance;
  private boolean balancing = false;

  // CAN IDs of the arm motors on the robot.
  // Assigned when we had 8 CAN motors. We switched
  // back to PWM to change to a Tank drivetrain from
  // Mecanum to climb the charge station
  private static final int armPivotCANID = 5;
  private static final int armExtensionCANID = 8;

  // Drivetrain Motors
  private PWMSparkMax leftMotor =  new PWMSparkMax(2);
  private PWMSparkMax rightMotor =  new PWMSparkMax(3);
  private DifferentialDrive driveTrain;

  // Drivetrain speeds - pressing right trigger slows it down
  // Fast driving speed
  private double driveFast = 1.0;
  // Slow driving speed
  private double driveSlow = .4;
  // Scaling factor to slow down drive speed
  private double driveScale = driveFast;
  // Autobalance speed - not using currently
  private double autoBalanceSpeed;
  // Autonomous routine menu
  //private final SendableChooser<String> autoMenu = new SendableChooser<>();
  private final String mobility = "Mobility";
  private final String dock = "Dock";
  private String autoSelected;
  // Autonomous speed
  private String autoSpeed;

  // Intake Motors
  private double leftIntakeDirection = -1.0;
  private double rightIntakeDirection = 1.0;
  // Slow down intake release
  private double intakeScale = 1.0;
  private PWMSparkMax leftIntakeMotor =  new PWMSparkMax(4);
  private PWMSparkMax rightIntakeMotor =  new PWMSparkMax(5);

  // Arm Motors
  private CANSparkMax armExtensionMotor = new CANSparkMax(armExtensionCANID, MotorType.kBrushless);
  private RelativeEncoder armExtensionEncoder;
  private SparkMaxPIDController armExtensionPIDController;
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
  public double kD = .0009; 
  // I Zone (rotations) - limits when the I value starts keeping track
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
  public int armFloorRotations = 46;
  // Grab an upright cone at the top
  public int armConeTopRotations = -20;
  // Go to scoring angle
  public int armScoreRotations = 14;
  // Raise arm high for travelS
  public int armTravelRotations = 2;


  // Xbox controller for driver
  private XboxController xbox_drive;

  // Xbox controller for operator
  private XboxController xbox_operator;

  // Top Camera
  UsbCamera topcam;

  // LED Lights Controller
  private final Spark lights = new Spark(1);

  // AUTONOMOUS CODE
  @Override

  public void autonomousInit() 
  {
    // Timer reset
    autoTimer.reset();
    autoTimer.start();
        // Turn on the lights for autonomous based on our alliance color
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
      // Color waves, Rainbow Palette
      lights.set(-0.45);
    }
  }

  @Override
  public void autonomousPeriodic()
  {
    
    // Display match data on dashboard
    // We do this in periodic because robot code can start before connecting to the FMS
    if (DriverStation.isFMSAttached() && !hasMatchName) 
    {
      String eventName = DriverStation.getEventName();
      DriverStation.MatchType matchType = DriverStation.getMatchType();
      int matchNumber = DriverStation.getMatchNumber();
      int replayNumber = DriverStation.getReplayNumber();
      SmartDashboard.putString("MATCH", String.format("%s_%s_%d_%d", eventName, matchType.name(), matchNumber, replayNumber));
      hasMatchName = true;
    }    
    // PREVIOUS AUTONOMOUS CODE USED ON 3-31-23.
    // UNCOMMENT THIS SECTION AND COMMENT OUT THE NEW
    // SECTION IF WE NEED TO RESTORE.
    /*
    if (autoTimer.get() > 0 && autoTimer.get() < 4) 
    {
      armPivotPIDController.setP(kP);
      armPivotPIDController.setI(kI);
      armPivotPIDController.setD(kD);
      armPivotPIDController.setIZone(kIz);
      armPivotPIDController.setFF(kFF);
      armPivotPIDController.setOutputRange(kMinOutput, kMaxOutput);
      armPivotPIDController.setReference(armScoreRotations, CANSparkMax.ControlType.kPosition);
      armExtensionMotor.set(-.4);
    }

    if (autoTimer.get() > 4 && autoTimer.get() < 7) 
    {
      armExtensionMotor.set(0);
      // Drop game piece
      leftIntakeMotor.set(-.4 * leftIntakeDirection);
      rightIntakeMotor.set(-.4 * rightIntakeDirection);
    }

    if (autoTimer.get() > 7 && autoTimer.get() < 10) 
    {
      leftIntakeMotor.set(0);
      rightIntakeMotor.set(0);
      armExtensionMotor.set(.4);
      armPivotPIDController.setP(kP);
      armPivotPIDController.setI(kI);
      armPivotPIDController.setD(kD);
      armPivotPIDController.setIZone(kIz);
      armPivotPIDController.setFF(kFF);
      armPivotPIDController.setOutputRange(kMinOutput, kMaxOutput);
      armPivotPIDController.setReference(armTravelRotations, CANSparkMax.ControlType.kPosition);
    }
    if (autoTimer.get() > 11.0 && autoTimer.get() < 13.24) 
    {
      armExtensionMotor.set(0);
      // roll back
      leftMotor.set(-.3);
      rightMotor.set(.3);
    }      
    else if (autoTimer.get() > 13.24)
    {
      leftMotor.set(0);
      rightMotor.set(0);
    }
    */
    // NEW AUTONOMOUS CODE FOR 4-1-23 TO TRY AND AUTOBALANCE.
    // IF WE CAN'T GET THIS WORKING, COMMENT IT OUT AND UNCOMMENT
    // THE ABOVE SECTION.
    if (autoTimer.get() > 0 && autoTimer.get() < 4) 
    {
      armPivotPIDController.setP(kP);
      armPivotPIDController.setI(kI);
      armPivotPIDController.setD(kD);
      armPivotPIDController.setIZone(kIz);
      armPivotPIDController.setFF(kFF);
      armPivotPIDController.setOutputRange(kMinOutput, kMaxOutput);
      armPivotPIDController.setReference(armScoreRotations, CANSparkMax.ControlType.kPosition);
      armExtensionMotor.set(-.4);
    }

    if (autoTimer.get() > 4 && autoTimer.get() < 7) 
    {
      armExtensionMotor.set(0);
      // Drop game piece
      leftIntakeMotor.set(-.4 * leftIntakeDirection);
      rightIntakeMotor.set(-.4 * rightIntakeDirection);
    }

    if (autoTimer.get() > 7 && autoTimer.get() < 10) 
    {
      leftIntakeMotor.set(0);
      rightIntakeMotor.set(0);
      armExtensionMotor.set(.4);
      armPivotPIDController.setP(kP);
      armPivotPIDController.setI(kI);
      armPivotPIDController.setD(kD);
      armPivotPIDController.setIZone(kIz);
      armPivotPIDController.setFF(kFF);
      armPivotPIDController.setOutputRange(kMinOutput, kMaxOutput);
      armPivotPIDController.setReference(armTravelRotations, CANSparkMax.ControlType.kPosition);
      // START ROLLING BACK EARLY AND SLOW! .1 may not be enough power. .3 will probably be too much.
      // CHANGE THE VARIABLE slowSpeed below and it will set it for the whole scenario.
      private final slowSpeed = .1;
      
      leftMotor.set(-slowSpeed);
      rightMotor.set(slowSpeed);
    }
    if (autoTimer.get() > 11.0 && autoTimer.get() < 14.5) 
    {
      armExtensionMotor.set(0);
      // See if we're tilting and need to correct.
      // Starting with 6 and -6 as the downhill angles. We need to verify
      // on the practice charging station!!!!!
      if (autoBalance.getTilt() > 6)
      {
        balancing = true;
        // Roll forward
        leftMotor.set(slowSpeed);
        rightMotor.set(-slowSpeed);
      }
      else if (autoBalance.getTilt() < -6)
      {
        balancing = true;
        leftMotor.set(-slowSpeed);
        rightMotor.set(slowSpeed);
      }
      else
      {
        // Check if we were ever on the ramp and then stop if we're balanced.
        // Otherwise keep rolling back for mobility.
        if (balancing)
        {
          // If our tilt is between 6 and -6, then consider us level and stop.
          leftMotor.set(0);
          rightMotor.set(0); 
        }
      }
    }      
    else if (autoTimer.get() > 14.5)
    {
      // Stop no matter what 
      leftMotor.set(0);
      rightMotor.set(0);
    }
    // STOP COMMENTING OUT HERE!
  }

  // ROBOT INITIALIZATION
  @Override
  public void robotInit() 
  { 
    // Start data logging
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    // Set the match name to practice initially
    SmartDashboard.putString("MATCH", "practice");

    // AutoBalance engine - not using
    autoBalance = new AutoBalance();

    // Auto 
    // autoMenu.setDefaultOption("Mobility", mobility);
    // autoMenu.addOption("Dock", dock);
    // SmartDashboard.putData("Autonomous", autoMenu);
    
    // Set up Arm Extension encoder even though we aren't using it right now
    armExtensionEncoder = armExtensionMotor.getEncoder();
    armExtensionPIDController = armExtensionMotor.getPIDController();

    // We ARE using the armPivot encoder and doing a PIDController
    armPivotEncoder = armPivotMotor.getEncoder();
    armPivotPIDController = armPivotMotor.getPIDController();

    // Robot drivetrain subsystem
    driveTrain =  new DifferentialDrive(leftMotor, rightMotor);
    // XBox Controllers
    xbox_drive = new XboxController(0);
    xbox_operator = new XboxController(1);

    // Start drive camera
    topcam = CameraServer.startAutomaticCapture(0);

    // Turn on the lights
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue)
    {
      // Breath, Blue
      lights.set(-0.15); 
      SmartDashboard.putString("ALLIANCE", "BLUE");
    }
    else if (DriverStation.getAlliance() == DriverStation.Alliance.Red)
    {
      // Breath, Red
      lights.set(-0.17);
      SmartDashboard.putString("ALLIANCE", "RED");
    }
    else
    {
      // Color Waves, Rainbow Palette
      lights.set(-0.45);
    }

    SmartDashboard.putNumber("GAME CLOCK", DriverStation.getMatchTime());
    SmartDashboard.putNumber("ROBOT TILT", autoBalance.getTilt());
  }

  @Override
  // TELEOP 
  public void teleopPeriodic() 
  {
    // The section executes repeatedly when the robot enters TeleOp mode
    SmartDashboard.putNumber("ROBOT TILT", autoBalance.getTilt());
    // Slow down robot for precision driving
    if (xbox_drive.getRightTriggerAxis() > 0)
    {
      driveScale = driveSlow;
      System.out.println("Driving SLOW....");
    }
    else
    {
      driveScale = driveFast;
      //System.out.println("Driving FAST....");
    }

    // Left joystick controls driving and strafing in any direction.
    // Right joystick controls spinning in place
    if (!balancing)
    {
      driveTrain.arcadeDrive(xbox_drive.getRightX()*driveScale, xbox_drive.getLeftY()*driveScale);
    }
    // Arm extension
    armExtensionMotor.set(xbox_operator.getLeftY()*.6);

    // Display the arm Pivot encoder position on the dashboard (rotations)
    SmartDashboard.putNumber("Arm Pivot", armPivotEncoder.getPosition());
    SmartDashboard.putNumber("Arm Extension", armExtensionEncoder.getPosition());
    SmartDashboard.putNumber("Arm Pivot Encoder Velocity", armPivotEncoder.getVelocity());

    // Autobalance during teleop
    if (xbox_drive.getAButton())
    {
      if (!balancing)
      {
        autoBalanceSpeed = autoBalance.autoBalanceRoutine();
        leftMotor.set(autoBalanceSpeed);
        rightMotor.set(-autoBalanceSpeed);
      }
      else
      {
        balancing = true;
      }
    }
    else
    {
      balancing = false;
    }
    
    // Raise arm all the way to travel
    if (xbox_operator.getAButton())
    {
      System.out.println("A Button - Extend Arm for floor cone!");
      // Want this to extend the arm 4 in. but the encoder isn't working.
    }
    // Move arm into scoring position
    else if (xbox_operator.getBButton())
    {
      System.out.println("B Button - Arm at floor level!");
      armPivotPIDController.setP(kP);
      armPivotPIDController.setI(kI);
      armPivotPIDController.setD(kD);
      armPivotPIDController.setIZone(kIz);
      armPivotPIDController.setFF(kFF);
      armPivotPIDController.setOutputRange(kMinOutput, kMaxOutput);
      armPivotPIDController.setReference(armFloorRotations, CANSparkMax.ControlType.kPosition);
    } 
    // Move arm to the top of an upright cone
    else if (xbox_operator.getXButton())
    {
      System.out.println("X Button - Arm in Travel Mode!");
      armPivotPIDController.setP(kP);
      armPivotPIDController.setI(kI);
      armPivotPIDController.setD(kD);
      armPivotPIDController.setIZone(kIz);
      armPivotPIDController.setFF(kFF);
      armPivotPIDController.setOutputRange(kMinOutput, kMaxOutput);
      armPivotPIDController.setReference(armTravelRotations, CANSparkMax.ControlType.kPosition);
    } 
    // Lower arm to floor to pick up game pieces
    else if (xbox_operator.getYButton())
    {
      System.out.println("Y Button - Arm at scoring level!");
      armPivotPIDController.setP(kP);
      armPivotPIDController.setI(kI);
      armPivotPIDController.setD(kD);
      armPivotPIDController.setIZone(kIz);
      armPivotPIDController.setFF(kFF);
      armPivotPIDController.setOutputRange(kMinOutput, kMaxOutput);
      armPivotPIDController.setReference(armScoreRotations, CANSparkMax.ControlType.kPosition);
    } 

    // Intake system
    if (xbox_operator.getRightY() < 0)
    {
      intakeScale = .6;
    }
    else
    {
      intakeScale = 1.0;
    }
    if (xbox_operator.getRightTriggerAxis() > 0)
    {
      intakeScale = 1.0;
      System.out.println("Intake Fast");
    }
    leftIntakeMotor.set(xbox_operator.getRightY()  * leftIntakeDirection * intakeScale);
    rightIntakeMotor.set(xbox_operator.getRightY() * rightIntakeDirection * intakeScale);
  }


}

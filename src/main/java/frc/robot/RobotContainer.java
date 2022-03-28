// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import java.util.HashMap;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Axis;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.commands.AlignWithLimelightCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ShootyCommand;
import frc.robot.commands.ToggleIntakePistonsCommand;
import frc.robot.commands.autonomous.AroundTarmacCommand;
import frc.robot.commands.autonomous.ChargeCommand;
import frc.robot.commands.autonomous.CurveCommand;
import frc.robot.commands.autonomous.ShootBackLeftCommand;
import frc.robot.commands.autonomous.ShootBackRightCommand;
import frc.robot.commands.autonomous.ShootStraightBackCommand;
import frc.robot.common.Odometry;
import frc.robot.common.TrajectoryLoader;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final RobotBase robot;

  // NETWORK TABLES
  private final Field2d field = new Field2d();
  private final Timer timer = new Timer();

  // DRIVE
  public final CANSparkMax frontLeftMotor = new CANSparkMax(DriveTrain.FRONT_LEFT_MOTOR, MotorType.kBrushless);
  public final CANSparkMax frontRightMotor = new CANSparkMax(DriveTrain.FRONT_RIGHT_MOTOR, MotorType.kBrushless);
  public final CANSparkMax rearLeftMotor = new CANSparkMax(DriveTrain.REAR_LEFT_MOTOR, MotorType.kBrushless);
  public final CANSparkMax rearRightMotor = new CANSparkMax(DriveTrain.REAR_RIGHT_MOTOR, MotorType.kBrushless);


  // ODOMETRY
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
  private final NetworkTable table = ntInstance.getTable("/components/drivetrain");
  private final NetworkTableEntry slowModeEntry = table.getEntry("slow_mode");
  private final NetworkTableEntry slowRotationEntry = table.getEntry("slow_rotation");

  private final RelativeEncoder leftEncoder = frontLeftMotor.getEncoder();
  private final RelativeEncoder rightEncoder = frontRightMotor.getEncoder();

  private final DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  private final Odometry odometry = new Odometry(gyro, driveOdometry, leftEncoder, rightEncoder);

  // NAVX
  private final AHRS navx = new AHRS(SPI.Port.kMXP);

  // DRIVE SUBSYSTEM
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeftMotor, rearLeftMotor);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(frontRightMotor, rearRightMotor);

  private final DifferentialDrive driveTrain = new DifferentialDrive(leftMotors, rightMotors);
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(driveTrain, leftMotors, rightMotors, odometry, field, timer);

  private final double xSpeedMultiplierNormal = 0.7;
  private final double xRotationMultiplierNormal = 0.7;

  private final double xSpeedMultiplierSlow = xSpeedMultiplierNormal * 0.7;
  private final double xRotationMultiplierSlow = xRotationMultiplierNormal * 0.7;

  private boolean slowSpeedEnabled = false;
  private boolean slowRotationEnabled = false;

  // SHOOTER SUBSYSTEM
  private final CANSparkMax shooterMotor = new CANSparkMax(Shooter.SHOOTER_MOTOR, MotorType.kBrushless);
  private final DoubleSolenoid shooterSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Shooter.SHOOTER_SOLENOID_FWD, Shooter.SHOOTER_SOLENOID_REV);
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(shooterMotor, shooterSolenoid);

  // CLIMB SUBSYSTEM
  private final TalonFX climberMotor = new TalonFX(Climber.CLIMBER_MOTOR);
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem(climberMotor);

  // STORAGE SUBSYSTEM
  private final CANSparkMax storageMotorRight = new CANSparkMax(Storage.STORAGE_RIGHT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax storageMotorLeft = new CANSparkMax(Storage.STORAGE_LEFT_MOTOR, MotorType.kBrushless);
  private final MotorControllerGroup storageMotorGroup = new MotorControllerGroup(storageMotorLeft, storageMotorRight);
  private final StorageSubsystem storageSubsystem = new StorageSubsystem(storageMotorGroup); 
 
  // INTAKE SUBSYSTEM
  private final CANSparkMax intakeMotor = new CANSparkMax(Intake.INTAKE_MOTOR, MotorType.kBrushed);
  private final DoubleSolenoid leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Intake.SOLENOID_LEFT_FWD, Intake.SOLENOID_LEFT_REV);
  private final DoubleSolenoid rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Intake.SOLENOID_RIGHT_FWD, Intake.SOLENOID_RIGHT_REV);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(intakeMotor, leftSolenoid, rightSolenoid);

  // LIMELIGHT
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
 
  // TRAJECTORIES
  private final TrajectoryLoader trajectoryLoader = new TrajectoryLoader();
  private final HashMap<String, Trajectory> trajectories = trajectoryLoader.loadTrajectories();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(RobotBase robot) {
    this.robot = robot;
    // Configure the button bindings
    configureObjects();
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Joystick leftJoystick = new Joystick(0);
    Joystick rightJoystick = new Joystick(1);
    PS4Controller altJoystick = new PS4Controller(2);

    JoystickButton btnIntakeOut = new JoystickButton(altJoystick, 3);
    JoystickButton btnIntakeSolenoid = new JoystickButton(altJoystick, 4);
    
    JoystickButton btnStorageIn = new JoystickButton(altJoystick, 5);
    JoystickButton btnStorageOut = new JoystickButton(altJoystick, 1);
    
    JoystickButton btnShooterSpinLow = new JoystickButton(altJoystick, 6);

    JoystickButton btnClimberMiddle = new JoystickButton(rightJoystick, 3);
    JoystickButton btnClimberDown = new JoystickButton(rightJoystick, 1);
    JoystickButton btnClimberManualDown = new JoystickButton(rightJoystick, 2);
    JoystickButton btnClimberManualUp = new JoystickButton(rightJoystick, 4);
    JoystickButton btnClimberZero = new JoystickButton(leftJoystick, 2);
    
    JoystickButton btnSlowMode = new JoystickButton(leftJoystick, 1);
    JoystickButton btnSlowRotation = new JoystickButton(leftJoystick, 2);

    JoystickButton btnAlign = new JoystickButton(leftJoystick, 1);


    // DRIVETRAIN
    driveSubsystem.setDefaultCommand(new RunCommand(
        () -> {
          if (robot.isTeleopEnabled()) {
            driveSubsystem.joystickDrive(leftJoystick.getY() * getSpeedMultiplier(), rightJoystick.getX() * getRotationMultiplier());
          } else {
            driveSubsystem.drive(0, 0);
          }
        },
        driveSubsystem));

    btnSlowMode.whenPressed(new InstantCommand(() -> {
      // makes both rotation and speed slower
      slowSpeedEnabled = !slowSpeedEnabled;
      slowRotationEnabled = !slowRotationEnabled;
      slowModeEntry.setBoolean(slowSpeedEnabled);
    })); 

    btnSlowRotation.whenPressed(new InstantCommand(() -> {
      // just makes rotation slower
      slowRotationEnabled = !slowRotationEnabled;
      slowRotationEntry.setBoolean(slowRotationEnabled);
    }));

    btnAlign
        .whenHeld(new AlignWithLimelightCommand(limelightSubsystem, driveSubsystem));


    // INTAKE
    intakeSubsystem.setDefaultCommand(new RunCommand(
        () -> intakeSubsystem.spin(DriverValues.intakeInVoltage * altJoystick.getRawAxis(2)), intakeSubsystem));

    btnIntakeOut
      .whileHeld(new InstantCommand(() -> intakeSubsystem.spin(DriverValues.intakeOutVoltage), intakeSubsystem))
      .whenInactive(new InstantCommand(() -> intakeSubsystem.spin(0), intakeSubsystem), true);

    btnIntakeSolenoid.toggleWhenPressed(new ToggleIntakePistonsCommand(intakeSubsystem));
    
    btnIntakeSolenoid
      .whileHeld(new InstantCommand(() -> intakeSubsystem.spin(DriverValues.intakeInVoltage), intakeSubsystem))
      .whenInactive(new InstantCommand(() -> intakeSubsystem.spin(0), intakeSubsystem), true);

    
    // SHOOTER
    shooterSubsystem.setDefaultCommand(new RunCommand(
        () -> shooterSubsystem.shootVelocity(DriverValues.shooterHighVelocity * altJoystick.getRawAxis(3)), shooterSubsystem));
    
    btnShooterSpinLow
      .whileHeld(new InstantCommand(() -> shooterSubsystem.shootVelocity(DriverValues.shooterLowVelocity), shooterSubsystem))
      .whenReleased(new InstantCommand(() -> shooterSubsystem.shootVoltage(0), shooterSubsystem), true);

    
    // STORAGE
    btnStorageIn
      .whileHeld(new InstantCommand(() -> storageSubsystem.spinVolts(DriverValues.storageInVoltage), storageSubsystem))
      .whenInactive(new InstantCommand(() -> storageSubsystem.spinVolts(0), storageSubsystem), true);
    
    btnStorageOut
      .whileHeld(new InstantCommand(() -> {
        storageSubsystem.spinVolts(DriverValues.storageOutVoltage);
      }, storageSubsystem))
      .whenInactive(new InstantCommand(() -> {
        storageSubsystem.spinVolts(0);
      }, storageSubsystem), true);

    btnStorageOut.whileHeld(new InstantCommand(() -> {
        shooterSubsystem.shootVoltage(3);
      }, shooterSubsystem))
      .whenInactive(new InstantCommand(() -> {
        shooterSubsystem.shootVoltage(0);
      }, shooterSubsystem), true);
    
    // CLIMBER
    btnClimberMiddle
      .whileHeld(new InstantCommand(() -> climbSubsystem.setWinchPos(Climber.MEDIUM_RUNG_POS), climbSubsystem))
      .whenReleased(new InstantCommand(() -> climbSubsystem.setWinchMotor(0), climbSubsystem), true);

    btnClimberManualDown
      .whileHeld(new InstantCommand(() -> climbSubsystem.setWinchMotor(DriverValues.climberDownVoltage), climbSubsystem))
      .whenReleased(new InstantCommand(() -> climbSubsystem.setWinchMotor(0), climbSubsystem), true);

    btnClimberManualUp
      .whileHeld(new InstantCommand(() -> climbSubsystem.setWinchMotor(DriverValues.climberUpVoltage), climbSubsystem))
      .whenReleased(new InstantCommand(() -> climbSubsystem.setWinchMotor(0), climbSubsystem), true);

    btnClimberZero
      .whenHeld(new InstantCommand(() -> climbSubsystem.resetClimberPosition()));

    btnClimberDown
      .whileHeld(new InstantCommand(() -> climbSubsystem.setWinchPos(Climber.CLIMBER_DOWN_POS), climbSubsystem))
      .whenReleased(new InstantCommand(() -> climbSubsystem.setWinchMotor(0), climbSubsystem), true);

  }

  private double getSpeedMultiplier()
  {
    if (slowSpeedEnabled)
      return xSpeedMultiplierSlow;
    else
      return xSpeedMultiplierNormal;
  }

  private double getRotationMultiplier()
  {
    if (slowRotationEnabled)
      return xRotationMultiplierSlow;
    else
      return xRotationMultiplierNormal;
  }
  
  public void configureObjects() {
    frontRightMotor.setIdleMode(IdleMode.kBrake);
    frontLeftMotor.setIdleMode(IdleMode.kBrake);
    rearRightMotor.setIdleMode(IdleMode.kBrake);
    rearLeftMotor.setIdleMode(IdleMode.kBrake);

    frontRightMotor.setInverted(true);
    frontLeftMotor.setInverted(false);
    rearRightMotor.setInverted(true);
    rearLeftMotor.setInverted(false);

    shooterMotor.setIdleMode(IdleMode.kCoast);
    
    storageMotorLeft.setInverted(true);
    storageMotorRight.setInverted(false);
    
    odometry.zeroHeading();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    odometry.zeroHeading();
    // return new ShootStraightBackCommand(driveSubsystem, odometry, storageSubsystem, shooterSubsystem, trajectories);
    // return new ShootBackRightCommand(driveSubsystem, odometry, storageSubsystem, shooterSubsystem, trajectories);
    return new ShootBackLeftCommand(driveSubsystem, odometry, storageSubsystem, shooterSubsystem, trajectories);
    // return new ShootyCommand(-2100, 2.5, 3, 8, shooterSubsystem, storageSubsystem);
  }
  
  public Timer getTimer() {
    return timer;
  }

  public Odometry getOdometry() {
    return odometry;
  }
}
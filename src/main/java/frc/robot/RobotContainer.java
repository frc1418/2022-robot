// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ToggleIntakePistonsCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
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

  // DRIVE
  private final CANSparkMax frontLeftMotor = new CANSparkMax(DriveTrain.FRONT_LEFT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax frontRightMotor = new CANSparkMax(DriveTrain.FRONT_RIGHT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax rearLeftMotor = new CANSparkMax(DriveTrain.REAR_LEFT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax rearRightMotor = new CANSparkMax(DriveTrain.REAR_RIGHT_MOTOR, MotorType.kBrushless);

  // DRIVE SUBSYSTEM
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeftMotor, rearLeftMotor);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(frontRightMotor, rearRightMotor);

  private final DifferentialDrive driveTrain = new DifferentialDrive(leftMotors, rightMotors);
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(driveTrain);

  private final double xSpeedMultiplierNormal = 0.6;
  private final double xRotationMultiplierNormal = 0.45;

  private final double xSpeedMultiplierSlow = xSpeedMultiplierNormal * 0.2;
  private final double xRotationMultiplierSlow = xRotationMultiplierNormal * 0.2;

  private double xSpeedMultiplier = xSpeedMultiplierNormal;
  private double xRotationMultiplier = xRotationMultiplierNormal;

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
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(RobotBase robot) {
    this.robot = robot;
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    frontRightMotor.setInverted(true);
    storageMotorRight.setInverted(true);

    Joystick leftJoystick = new Joystick(0);
    Joystick rightJoystick = new Joystick(1);
    Joystick altJoystick = new Joystick(2);

    JoystickButton btnIntakeIn = new JoystickButton(altJoystick, 4);
    JoystickButton btnIntakeOut = new JoystickButton(altJoystick, 6);
    JoystickButton btnIntakeSolenoid = new JoystickButton(altJoystick, 2);
    
    JoystickButton btnStorageIn = new JoystickButton(altJoystick, 3);
    JoystickButton btnStorageOut = new JoystickButton(altJoystick, 5);
    
    JoystickButton btnShooterSpin = new JoystickButton(altJoystick, 9);
    JoystickButton btnShooterSolenoid = new JoystickButton(altJoystick, 1);

    JoystickButton btnClimberUp = new JoystickButton(leftJoystick, 2);
    JoystickButton btnClimberDown = new JoystickButton(rightJoystick, 2);
    
    JoystickButton btnSlowMode = new JoystickButton(leftJoystick, 1);
    JoystickButton btnSlowRotation = new JoystickButton(rightJoystick, 1);

    driveSubsystem.setDefaultCommand(new RunCommand(
        () -> {
          if (robot.isTeleopEnabled()) {
            driveSubsystem.joystickDrive(leftJoystick.getY() * xSpeedMultiplier, rightJoystick.getX() * xRotationMultiplier);
          } else {
            driveSubsystem.drive(0, 0);
          }
        },
        driveSubsystem));
    
    btnIntakeIn
      .whileHeld(new InstantCommand(() -> intakeSubsystem.spin(-7), intakeSubsystem))
      .whenInactive(new InstantCommand(() -> intakeSubsystem.spin(0), intakeSubsystem), true);

    btnIntakeOut
      .whileHeld(new InstantCommand(() -> intakeSubsystem.spin(5), intakeSubsystem))
      .whenInactive(new InstantCommand(() -> intakeSubsystem.spin(0), intakeSubsystem), true);

    btnIntakeSolenoid.toggleWhenPressed(new ToggleIntakePistonsCommand(intakeSubsystem));
      
    btnStorageIn
      .whileHeld(new InstantCommand(() -> storageSubsystem.spinVolts(2.5), storageSubsystem))
      .whenInactive(new InstantCommand(() -> storageSubsystem.spinVolts(0), storageSubsystem), true);
    
    btnStorageOut
      .whileHeld(new InstantCommand(() -> storageSubsystem.spinVolts(-1.5), storageSubsystem))
      .whenInactive(new InstantCommand(() -> storageSubsystem.spinVolts(0), storageSubsystem), true);

    btnShooterSpin
      .whenHeld(new InstantCommand(() -> shooterSubsystem.shootVelocity(Shooter.TARMAC_LINE_VEL), shooterSubsystem))
      .whenReleased(new InstantCommand(() -> shooterSubsystem.shootVoltage(0), shooterSubsystem), true);

    btnShooterSolenoid
      .whenHeld(new InstantCommand(() -> shooterSubsystem.setPiston(DoubleSolenoid.Value.kForward), shooterSubsystem))
      .whenReleased(new InstantCommand(() -> shooterSubsystem.setPiston(DoubleSolenoid.Value.kReverse), shooterSubsystem), true);

    btnClimberUp
      .whenHeld(new InstantCommand(() -> climbSubsystem.setWinchMotor(0), climbSubsystem))
      .whenReleased(new InstantCommand(() -> climbSubsystem.setWinchMotor(0), climbSubsystem), true);

    btnClimberDown
      .whenHeld(new InstantCommand(() -> climbSubsystem.setWinchMotor(0), climbSubsystem))
      .whenReleased(new InstantCommand(() -> climbSubsystem.setWinchMotor(0), climbSubsystem), true);

    // makes both rotation and speed slower
    btnSlowMode.whenPressed(new InstantCommand(() -> {
      if (xSpeedMultiplier == xSpeedMultiplierNormal)
      {
        xSpeedMultiplier = xSpeedMultiplierSlow;
        xRotationMultiplier = xRotationMultiplierSlow;
      }
      else
      {
        xSpeedMultiplier = xSpeedMultiplierNormal;
        xRotationMultiplier = xRotationMultiplierNormal;
      }
    })); 

    // just makes rotation slower
    btnSlowRotation.whenPressed(new InstantCommand(() -> {
      if (xRotationMultiplier == xRotationMultiplierNormal)
        xRotationMultiplier = xRotationMultiplierSlow;
      else
        xRotationMultiplier = xRotationMultiplierNormal;
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
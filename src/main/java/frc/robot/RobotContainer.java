// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

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
  private double xSpeedMultiplier = 0.6;

  // DRIVE SUBSYSTEM
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeftMotor, rearLeftMotor);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(frontRightMotor, rearRightMotor);

  private final DifferentialDrive driveTrain = new DifferentialDrive(leftMotors, rightMotors);
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(driveTrain);

  // SHOOTER SUBSYSTEM
  private final CANSparkMax shooterMotor = new CANSparkMax(Shooter.SHOOTER_MOTOR, MotorType.kBrushless);
  private final ShooterSubsystem shooter = new ShooterSubsystem(shooterMotor);

  //CLIMB SUBSYSTEM
  private final Talon climberMotor = new Talon(Climber.CLIMBER_MOTOR);
  private final ClimbSubsystem climber = new ClimbSubsystem(climberMotor);

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
    rightMotors.setInverted(true);

    Joystick leftJoystick = new Joystick(0);
    Joystick rightJoystick = new Joystick(1);
    Joystick altJoystick = new Joystick(2);

        driveSubsystem.setDefaultCommand(new RunCommand(
          () -> {
              if (robot.isTeleopEnabled()) {
                  driveSubsystem.joystickDrive(leftJoystick.getY() * xSpeedMultiplier, rightJoystick.getX() * 0.45);
              } else {
                  driveSubsystem.drive(0, 0);
              }
          },
          driveSubsystem));
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
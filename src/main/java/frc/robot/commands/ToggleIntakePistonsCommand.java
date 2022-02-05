package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntakePistonsCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;

    public ToggleIntakePistonsCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        this.intakeSubsystem.extend();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.retract();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
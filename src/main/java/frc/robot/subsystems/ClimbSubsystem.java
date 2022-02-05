package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

    private final TalonFX climberMotor;

    public ClimbSubsystem(TalonFX climberMotor) {
        this.climberMotor = climberMotor;
    }

    @Override
    public void periodic() {

    }

    public void setWinchMotor(double speed) {

    }
    
}

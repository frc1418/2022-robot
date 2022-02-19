package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

    private final TalonFX climberMotor;

    public ClimbSubsystem(TalonFX climberMotor) {
        this.climberMotor = climberMotor;
        climberMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {

    }

    public void setWinchMotor(double speed) {
        climberMotor.set(TalonFXControlMode.Velocity, speed);
    }

    public void setWinchPos(double pos){
        climberMotor.set(TalonFXControlMode.Position, pos);
    }
    
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

    private final Talon climberMotor;

    public ClimbSubsystem(Talon climberMotor){
        this.climberMotor = climberMotor;
    }

    @Override
    public void periodic(){

    }

    public void setWinchMotor(double speed){
        climberMotor.set(speed);
    }
    
}

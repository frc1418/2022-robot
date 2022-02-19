package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

    private final TalonFX climberMotor;

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/climber");
    private final NetworkTableEntry pos = table.getEntry("climber_pos");

    public ClimbSubsystem(TalonFX climberMotor) {
        this.climberMotor = climberMotor;
        climberMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {
        pos.setDouble(climberMotor.getSensorCollection().getIntegratedSensorPosition());
    }

    public void setWinchMotor(double speed) {
        climberMotor.set(TalonFXControlMode.Velocity, speed);
    }

    public void setWinchPos(double pos){
        climberMotor.set(TalonFXControlMode.Position, pos);
    }
    
}

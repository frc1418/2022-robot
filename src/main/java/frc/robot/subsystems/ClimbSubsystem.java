package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Climber;

public class ClimbSubsystem extends SubsystemBase {

    private final TalonFX climberMotor;

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/climber");
    private final NetworkTableEntry pos = table.getEntry("climber_pos");
    private final NetworkTableEntry targetPos = table.getEntry("target_pos");

    // used to convert from clicks to rotation
    public ClimbSubsystem(TalonFX climberMotor) {
        this.climberMotor = climberMotor;
        climberMotor.setNeutralMode(NeutralMode.Brake);
        climberMotor.selectProfileSlot(0, 1);
        climberMotor.config_kP(0, 0.024);
        climberMotor.config_kI(0, 0);
        climberMotor.config_kD(0, 0);
        // climberMotor.config_kF(0, 0.0006);

        targetPos.setDouble(0);
    }

    @Override
    public void periodic() {
        pos.setDouble(climberMotor.getSensorCollection().getIntegratedSensorPosition() / Climber.CLICKS_PER_ROTAITON);
    }

    public void setWinchMotor(double speed) {
        climberMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void setWinchPos(double pos){
        targetPos.setDouble(pos);

        climberMotor.set(TalonFXControlMode.Position, Climber.CLICKS_PER_ROTAITON * pos);
    }

    public double getCurrentPos(){
        return climberMotor.getSensorCollection().getIntegratedSensorPosition();
    }

    public void resetClimberPosition(){
        climberMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
        targetPos.setDouble(0);
    }
    
}

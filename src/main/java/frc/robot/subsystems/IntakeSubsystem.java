package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor;
    private final DoubleSolenoid leftSolenoid;
    private final DoubleSolenoid rightSolenoid;

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/intake");
    private final NetworkTableEntry piston_ext = table.getEntry("piston_extended");

    public IntakeSubsystem(
            CANSparkMax intakeMotor,
            DoubleSolenoid leftSolenoid,
            DoubleSolenoid rightSolenoid) {

        this.intakeMotor = intakeMotor;
        this.leftSolenoid = leftSolenoid;
        this.rightSolenoid = rightSolenoid;

        intakeMotor.setIdleMode(IdleMode.kCoast);
        retract();
    }

    public void retract() {
        piston_ext.setBoolean(true);
        leftSolenoid.set(DoubleSolenoid.Value.kForward);
        rightSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void extend() {
        piston_ext.setBoolean(false);
        leftSolenoid.set(DoubleSolenoid.Value.kReverse);
        rightSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void spin(double intakeVolts) {
        intakeMotor.setVoltage(intakeVolts);
    }

    public boolean isExtended(){
        return piston_ext.getBoolean(true);
    }

    @Override
    public void periodic() {
        
    }
 
}

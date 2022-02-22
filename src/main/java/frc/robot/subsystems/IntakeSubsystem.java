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

    private final NetworkTable table = ntInstance.getTable("/common/Intake");
    private final NetworkTableEntry pistons = table.getEntry("pistonsExtended");

    public IntakeSubsystem(
            CANSparkMax intakeMotor,
            DoubleSolenoid leftSolenoid,
            DoubleSolenoid rightSolenoid) {

        this.intakeMotor = intakeMotor;
        this.leftSolenoid = leftSolenoid;
        this.rightSolenoid = rightSolenoid;

        intakeMotor.setIdleMode(IdleMode.kCoast);
        this.retract();
    }

    public void extend() {
        leftSolenoid.set(DoubleSolenoid.Value.kReverse);
        rightSolenoid.set(DoubleSolenoid.Value.kReverse);
        pistons.setBoolean(true);
    }

    public void retract() {
        leftSolenoid.set(DoubleSolenoid.Value.kForward);
        rightSolenoid.set(DoubleSolenoid.Value.kForward);
        pistons.setBoolean(false);
    }

    public void spin(double intakeVolts) {
        intakeMotor.setVoltage(intakeVolts);
    }

    @Override
    public void periodic() {
        
    }
 
}

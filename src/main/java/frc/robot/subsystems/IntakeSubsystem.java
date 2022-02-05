package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor;
    private final DoubleSolenoid leftSolenoid;
    private final DoubleSolenoid rightSolenoid;

    public IntakeSubsystem(
            CANSparkMax intakeMotor,
            DoubleSolenoid leftSolenoid,
            DoubleSolenoid rightSolenoid) {

        this.intakeMotor = intakeMotor;
        this.leftSolenoid = leftSolenoid;
        this.rightSolenoid = rightSolenoid;

        intakeMotor.setIdleMode(IdleMode.kCoast);
    }

    public void extend() {
        leftSolenoid.set(DoubleSolenoid.Value.kReverse);
        rightSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void retract() {
        leftSolenoid.set(DoubleSolenoid.Value.kForward);
        rightSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void spin(double intakeVolts) {
        intakeMotor.setVoltage(intakeVolts);
    }

    @Override
    public void periodic() {
        
    }
 
}

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.SparkMaxPIDController;

public class ShooterSubsystem  extends SubsystemBase {
    
    private final CANSparkMax shooterMotor;
    private final SparkMaxPIDController shooterController;
    private final RelativeEncoder shooterEncoder;
    private final DoubleSolenoid shooterSolenoid;

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/launcher");
    private final NetworkTableEntry rpm = table.getEntry("filtered_rpm");
    private final NetworkTableEntry ntTargetRPM = table.getEntry("target_rpm");
    private final NetworkTableEntry output = table.getEntry("output");
    private final NetworkTableEntry current = table.getEntry("current");

    private double targetRPM = 0;


    public ShooterSubsystem(CANSparkMax shooterMotor, DoubleSolenoid shooterSolenoid) {
        this.shooterMotor = shooterMotor;
        this.shooterController = shooterMotor.getPIDController();
        this.shooterEncoder = shooterMotor.getEncoder();
        this.shooterSolenoid = shooterSolenoid;
    }

    public void shootVelocity(double shooterSpeed) {
        shooterController.setReference(shooterSpeed, ControlType.kVelocity);
        targetRPM = shooterSpeed;
    }

    public void shootVoltage(double voltage) {
        shooterMotor.setVoltage(voltage);
    }

    @Override
    public void periodic() {

        // sets networkTable values
        rpm.setDouble(this.shooterEncoder.getVelocity());
        ntTargetRPM.setDouble(targetRPM);
        output.setDouble(this.shooterMotor.getAppliedOutput());
        current.setDouble(this.shooterMotor.getOutputCurrent());
    }

    public void setPiston(DoubleSolenoid.Value value) {
        // if true, piston should extend
        shooterSolenoid.set(value);
    }
}

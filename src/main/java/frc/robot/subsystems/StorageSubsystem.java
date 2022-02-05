package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StorageSubsystem extends SubsystemBase {
    
    public final MotorControllerGroup storageMotorGroup;

    public StorageSubsystem(MotorControllerGroup storageMotorGroup) {
        this.storageMotorGroup = storageMotorGroup;
    }

    @Override
    public void periodic() {

    }

    public void run(double speed) {
        storageMotorGroup.set(speed);
    }
}

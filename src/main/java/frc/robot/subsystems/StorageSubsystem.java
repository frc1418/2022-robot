package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StorageSubsystem extends SubsystemBase {
    
    public final CANSparkMax storageMotorRight;
    public final CANSparkMax storageMotorLeft;
    public final MotorControllerGroup storageMotorGroup;

    public StorageSubsystem(CANSparkMax storageMotorRight, CANSparkMax storageMotorLeft) {
        this.storageMotorRight = storageMotorRight;
        this.storageMotorLeft = storageMotorLeft;
        storageMotorGroup = new MotorControllerGroup(storageMotorLeft, storageMotorRight);
    }

    @Override
    public void periodic(){

    }

    public void run(double speed){
        storageMotorGroup.set(speed);
    }
}

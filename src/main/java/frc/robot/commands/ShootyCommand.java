package frc.robot.commands;

import java.sql.Driver;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

// public class ShootyCommand extends CommandBase {

//     private final ShooterSubsystem shooterSubsystem;
//     private final double targetVel;
//     private final StorageSubsystem storageSubsystem;

//     private boolean shooting;
//     private boolean finished;

//     public ShootyCommand(double targetVel, ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem){
//         this.shooterSubsystem = shooterSubsystem;
//         this.targetVel = targetVel;
//         this.storageSubsystem = storageSubsystem;

//         shooting = false;
//         finished = false;
//     }


//     @Override
//     public void execute(){
//         shooterSubsystem.shootVelocity(targetVel);
//         System.out.println("SHOOTING: " + targetVel);
//         if(Math.abs(shooterSubsystem.getRPM() - targetVel) < 100){
//             shooterSubsystem.shootVelocity(targetVel);
//             System.out.println("SHOOTING WELL");
//             storageSubsystem.spinVolts(2.5);
//             shooting = true;
//         }else if (shooting) {
//             System.out.println("DONE");
//             finished = true;
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         shooterSubsystem.shootVelocity(0);
//         storageSubsystem.spinVolts(0);
//     }

//     @Override
//     public boolean isFinished() {
//         return finished;
//     }

// }


public class ShootyCommand extends SequentialCommandGroup {
    

    public ShootyCommand(double shooterTargetVel, double storageVolts, double shootTime, double shootTimeoutTime, ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem){

        addCommands(

            new ParallelCommandGroup(
                new InstantCommand(() -> shooterSubsystem.shootVelocity(shooterTargetVel), shooterSubsystem),

                new WaitUntilCommand(shooterSubsystem :: atSetpoint)
                    .andThen(
                        new ParallelCommandGroup(
                            new WaitCommand(shootTime),
                            new InstantCommand(() -> storageSubsystem.spinVolts(storageVolts))
                        )
                    )
            ).withTimeout(shootTimeoutTime)
            .andThen(
                () -> {
                    shooterSubsystem.shootVoltage(0);
                    storageSubsystem.spinVolts(0);
                })
            
            
            
        );

    }
}
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class GyroReset extends Command{
    // private final Runnable reset;
    SwerveSubsystem swerve;

    public GyroReset(SwerveSubsystem swerveSubsystem) {
        System.out.println("A side of fries");
        swerve = swerveSubsystem;
    
    }

    @Override
    public void initialize() {
        System.out.println("Two scoop custard with butterfinger");
        swerve.zeroGyro();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean runsWhenDisabled(){
        return true;
    }
}

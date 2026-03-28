package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Universals;
import frc.robot.subsystems.SwerveSubsystem;

public class BrakeCommand extends Command{
    private SwerveSubsystem swerve;
    // private boolean brake;

    public BrakeCommand(SwerveSubsystem swerve){
        this.swerve = swerve;
        // this.brake = brake;//lkajs;dlkajsd
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        swerve.lockpose();
        Universals.brakemode = true;
    }

    @Override
    public void end(boolean interrupted){
        Universals.brakemode = false;
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}

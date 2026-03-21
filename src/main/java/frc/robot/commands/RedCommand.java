package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleSubsystem;

public class RedCommand extends Command {
    private CANdleSubsystem candle;

    public RedCommand(CANdleSubsystem candle){
        this.candle = candle;
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        candle.redIt();
    }

    @Override
    public void end(boolean interrupted){
        candle.defaultColor();
    }

    }

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleSubsystem;

public class WhiteCommand extends Command {
    private CANdleSubsystem candle;

    public WhiteCommand(CANdleSubsystem candle){
        this.candle = candle;
    }

    @Override
    public void initialize(){
        
    }
    @Override
    public void execute(){
        // candle.whiteIt();
    }

    @Override
    public void end(boolean interrupted){
    }

    }


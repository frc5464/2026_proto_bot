package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Universals;

public class SlowModeCommand extends Command{
    @Override
    public void initialize() {
        // Enable slow mode while a button on the op controller is pressed
        Universals.slowMode = true;
    }

    @Override
    public void end(boolean interrupted) {
        // When button is released, turn off slow mode
        Universals.slowMode = false;
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public boolean runsWhenDisabled(){
        return true;
    }
}

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.BrakeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.GyroReset;
import frc.robot.commands.SlowModeCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  public SwerveSubsystem swerve = new SwerveSubsystem();
  
  private final CommandJoystick driveController = new CommandJoystick(0);
  
  public final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser); 
    
    FollowPathCommand.warmupCommand();
    
    configureBindings();

    //controller deadband for drive controller
        double driveX = driveController.getRawAxis(1);
        double driveY = driveController.getRawAxis(0);
        double driveRot = -driveController.getRawAxis(4);
        if(Math.abs(driveX) < 0.1){ driveX = 0;}
        if(Math.abs(driveY) < 0.1){ driveY = 0;}
        if(Math.abs(driveRot) < 0.1){ driveRot = 0;}

  }

  public void periodic(){

  }

  public void configureBindings(){
    // silence the unplugged controller message if we are simulating!
        if(Robot.isSimulation()){
            DriverStation.silenceJoystickConnectionWarning(true);
        }


      swerve.setDefaultCommand(new DriveCommand(swerve, driveController));
      driveController.button(1).whileTrue(new BrakeCommand(swerve));
      driveController.button(8).onTrue(new GyroReset(swerve));
      driveController.button(7).whileTrue(new SlowModeCommand());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
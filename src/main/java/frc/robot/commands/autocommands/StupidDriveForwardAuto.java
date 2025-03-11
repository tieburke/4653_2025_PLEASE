package frc.robot.commands.autocommands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.LimelightHelpers;
import frc.robot.RobotContainer;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class StupidDriveForwardAuto extends Command {    
    private Swerve s_Swerve;  
    private Timer timer; 

    public StupidDriveForwardAuto(Swerve s_Swerve){
        this.s_Swerve = s_Swerve; 
        this.timer = new Timer();
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        s_Swerve.zeroGyro();
    }

    @Override
    public void execute() {
            s_Swerve.drive(
                new Translation2d(0.8, 0), 
                0, 
                false, 
                true
            );
    }

      // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
        s_Swerve.drive(
            new Translation2d(0, 0),0, 
            false, 
            true
        );
    }

    @Override
    public boolean isFinished(){
        if(timer.get() > 1){
            return true;
        }
        return false;
    }
}
package frc.robot.commands.autocommands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.LimelightHelpers;
import frc.robot.RobotContainer;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class LimelightAlignAuto extends Command {    
    private Swerve s_Swerve;  
    private Timer timer; 

    public LimelightAlignAuto(Swerve s_Swerve){
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
                new Translation2d(0.2, RobotContainer.limelight_aim_proportional()).times(Constants.Swerve.maxSpeed), 
                0, 
                false, 
                true
            );
            LimelightHelpers.setPipelineIndex("limelight", 0);  
    }

      // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("FINISHED", isFinished());
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
        if(LimelightHelpers.getTA("limelight") > 22){
            return true;
        }
        else if(timer.get() > 4){
            return true;
        }
        else{
            return false;
        }
    }
}
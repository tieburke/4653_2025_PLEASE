package frc.robot.commands.autocommands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.LimelightHelpers;
import frc.robot.RobotContainer;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class LimelightAlignX extends Command {    
    private Swerve s_Swerve;  
    private Timer timer; 

    public LimelightAlignX(Swerve s_Swerve){
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
            //if(LimelightHelpers.getFiducialID("limelight") != -1){
            s_Swerve.drive(
                new Translation2d(0, RobotContainer.limelight_aim_proportional("limelight-b")).times(Constants.Swerve.maxSpeed), 
                0,//RobotContainer.limelight_range_proportional()*1.25, 
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
        if(Math.abs(LimelightHelpers.getTX("limelight-b")) < 2 && LimelightHelpers.getFiducialID("limelight-b") != -1){
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
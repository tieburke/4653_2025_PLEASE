package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase{
    
    private SparkMax winchMotor;
    private SparkMaxConfig mWinchConfig;
    private RelativeEncoder winchEncoder;

    private DigitalInput limitSwitch;

    public Elevator(){
        winchMotor = new SparkMax(Constants.Elevator.winchMotorID, MotorType.kBrushless);
        winchEncoder = winchMotor.getEncoder();
        limitSwitch = new DigitalInput(0);
        configElevator();
    }

    public void configElevator(){
        mWinchConfig = new SparkMaxConfig();
        mWinchConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(Constants.Elevator.winchMotorKP, Constants.Elevator.winchMotorKI, Constants.Elevator.winchMotorKD);
        winchMotor.configure(mWinchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        winchEncoder.setPosition(0);
    }

    public void setPosition(double position){
        winchMotor.getClosedLoopController().setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void elevatorUpManual(){
        winchMotor.set(-Constants.Elevator.manualSpeed);
    }

    public void elevatorDownManual(){
        winchMotor.set(Constants.Elevator.manualSpeed);
    }

    public void elevatorStop(){
        winchMotor.set(0);
    }

    public double getPosition(){
        return winchEncoder.getPosition();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Winch Encoder Value: ", winchEncoder.getPosition());
        SmartDashboard.putBoolean("Limit Switch: ", limitSwitch.get());
    }

}

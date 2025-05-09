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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private SparkMax climberMotor;
    private SparkMaxConfig mClimberConfig;
    private RelativeEncoder climberEncoder;

    public Climber() {
        climberMotor = new SparkMax(Constants.Climber.climberMotorID, MotorType.kBrushless);
        climberEncoder = climberMotor.getEncoder();
        configClimber();
    }

    public void configClimber() {
        mClimberConfig = new SparkMaxConfig();
        mClimberConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(Constants.Climber.climberMotorKP, Constants.Climber.climberMotorKI,
                        Constants.Climber.climberMotorKD);

        climberMotor.configure(mClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climberEncoder.setPosition(0);
    }

    public void setPosition(double position) {
        climberMotor.getClosedLoopController().setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void climberUpManual() {
        climberMotor.set(Constants.Climber.manualSpeed);
    }

    public void climberDownManual() {
        climberMotor.set(-Constants.Climber.manualSpeed);
    }

    public void climberStop() {
        climberMotor.set(0);
    }

    public void resetEncoder() {
        climberEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climber Encoder Value: ", climberEncoder.getPosition());
    }

}

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;

public final class CTREConfigs {

    public SparkMax swerveAngleMotor;
    public SparkMax swerveDriveMotor;

    public SparkClosedLoopController swerveAnglePidController;
    public SparkClosedLoopController swerveDrivePidController; 

    public SparkMaxConfig swerveAngleConfig;
    public SparkMaxConfig swerveDriveConfig;

    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveCanCoderConfig = new CANcoderConfiguration();

        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1; //apparently this sets it to the same range we had before
        swerveCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    }

    /* Pretty sure this is not needed */
    //TODO: see if im right

    // public void configAngle(SparkMax angleMotor){
    //     swerveAngleMotor = angleMotor;
    //     swerveAnglePidController = swerveAngleMotor.getClosedLoopController();
        // swerveAngleConfig = new SparkMaxConfig();

    //     /* Swerve Angle Motor Configurations */
    //     swerveAngleConfig.smartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit, Constants.Swerve.anglePeakCurrentLimit);

    //     //TODO: figure out these conversion factors
    //     swerveAngleConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

        // swerveAngleConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    //     .pidf(Constants.Swerve.angleKP, Constants.Swerve.angleKI, 
    //     Constants.Swerve.angleKD, Constants.Swerve.angleKF);

    //     //TODO: Figure out if this is needed
    //     //swerveAngleMotor.burnFlash();
    // }
    
    
    /* Pretty sure this is not needed */
    //TODO: see if im right
    
    // public void configDrive(SparkMax driveMotor){
    //     swerveDriveMotor = driveMotor;
    //     swerveDrivePidController = swerveDriveMotor.getClosedLoopController();
    //     swerveDriveConfig = new SparkMaxConfig();

    //     /* Swerve Angle Motor Configurations */
    //     swerveDriveConfig.smartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit, Constants.Swerve.drivePeakCurrentLimit);

    //     //TODO: figure out these conversion factors
    //     swerveDriveConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    //     swerveDriveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //     .pidf(Constants.Swerve.driveKP, Constants.Swerve.driveKI, 
    //     Constants.Swerve.driveKD, Constants.Swerve.driveKF);
        
    //     //TODO: Figure out if this is needed
    //     //swerveAngleMotor.burnFlash();
    // }

    public CANcoderConfiguration getCANConfig(){

        CANcoderConfiguration swerveCanCoderConfig = new CANcoderConfiguration();

        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        swerveCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        return swerveCanCoderConfig;
    }
}
package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.util.COTSNeoSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

    public static final double stickDeadband = 0.1;

    public static final class Swerve {

        public static final Translation2d flModuleOffset = new Translation2d(0.279, 0.279);
        public static final Translation2d frModuleOffset = new Translation2d(0.279, -0.279);
        public static final Translation2d blModuleOffset = new Translation2d(-0.279, 0.279);
        public static final Translation2d brModuleOffset = new Translation2d(-0.279, -0.279);

        // RPM -> M/S
        // (motor rotations / minute)*(1 minute / 60 seconds)*(1 wheel rotations / 8.14
        // motor rotations)*(0.6383 meters / wheel rotation)
        public static final double velocityConversionFactor = 0.00130706714;

        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        public static final COTSNeoSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
                COTSNeoSwerveConstants.SDSMK4(COTSNeoSwerveConstants.driveGearRatios.SDSMK4_L1);

        /* Drivetrain Constants */
        public static final double trackWidth = 0.279 * 2; // TODO: This must be tuned to specific robot
        public static final double wheelBase = 0.279 * 2; // TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 80;
        public static final int anglePeakCurrentLimit = 80;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive neo to ramp in open loop and closed loop
         * driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.1; // TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.326;

        /*
         * Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         */
        public static final double driveKS = (0.32 / 12); // TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot
        public static final double slowSpeed = .3;
        /** Radians per Second */
        public static final double maxAngularVelocity = 5; // 10 //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kCoast;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Module Specific Constants */

        // Align with bevel gears on right side

        /* Front Left Module - Module 0 */
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 20;
            public static Rotation2d angleOffset = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 21;
            public static Rotation2d angleOffset = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 22;
            public static Rotation2d angleOffset = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 23;
            public static Rotation2d angleOffset = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class Elevator {
        public static final int winchMotorID = 9;

        /*Old values*/
        public static final double winchMotorKP = 0.01;
        public static final double winchMotorKI = 0;
        public static final double winchMotorKD = 0.3;
        public static final double winchMotorFF = 0.002;

        // public static final double winchMotorKP = 0.011815;
        // public static final double winchMotorKI = 0;
        // public static final double winchMotorKD = 0.3;
         
        /* Drive Motor Characterization Values 
        * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double winchMotorKS = (4.8259 / 12);
        public static final double winchMotorKV = (0.019311 / 12);
        public static final double winchMotorKA = (0.0043659 / 12);

        public static final double L0Velo = 0.1;

        public static final double positionL0 = 0;
        public static final double positionL1 = 50*(9.0/64);
        public static final double positionL2 = 42*(9.0/64);
        public static final double positionL3 = 203*(9.0/64);
        public static final double positionL4 = 450*(9.0/64);

        public static final double manualSpeed = 0.5;
    }

    public static final class AlgaeIntake {
        // IDs for the intake motors
        public static final int leftIntakeID = 10;
        public static final int rightIntakeID = 11;
        public static final int articulateIntakeID = 12;

        public static final double articulateKP = 0.03;
        public static final double articulateKI = 0.0;
        public static final double articulateKD = 0.0;

        public static final double horizontalPos = -21.8;
        public static final double verticalPos = 0;

        public static final double manualSpeed = 0.15;
    }

    public static final class Climber {
        public static final int climberMotorID = 13;

        public static final double climberMotorKP = 0.05;
        public static final double climberMotorKI = 0;
        public static final double climberMotorKD = 0;
        
        public static final int positionClimb = 310;

        public static final double manualSpeed = 0.7;
    }

    public static final class RainGutter {
        public static final int rotateServoID = 0;
        public static final int releaseServoID = 1;
    }

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 8;
        public static final double kMaxAccelerationMetersPerSecondSquared = 8;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 0.0001;
        public static final double kIXController = 0;
        public static final double kDXController = 0;

        public static final double kPYController = 0.0001;
        public static final double kIYController = 0;
        public static final double kDYController = 0;

        public static final double kPThetaController = 0;
        public static final double kIThetaController = 0;
        public static final double kDThetaController = 0;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}

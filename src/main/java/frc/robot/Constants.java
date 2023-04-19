package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.04;



    
    public static final class Swerve {
        public static final int pigeonID = 0;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20.5); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(20.5); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
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
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = .1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = .1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = .35;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        //0.00082555 test 1 p value
        public static final double driveKP = 0.00082555; //TODO: This must be tuned to specific robot

       // public static final double driveKP = 0.01; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0;
        public static final double driveKF = 1;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.2479 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (6.9836 / 12);
        public static final double driveKA = (1.4986 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = .23; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = .35; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 0;
            public static final int canCoderID = 13;
            public static final int RelativeEncoder1 = 22;
            public static final int RelativeEncoder2 = 16;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees( 304.5);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 15;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 15;
            public static final int RelativeEncoder1 = 21;
            public static final int RelativeEncoder2 = 14;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(265);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID =2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 20;
            public static final int RelativeEncoder1 = 24;
            public static final int RelativeEncoder2 = 25;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(300.45);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 14;
            public static final int angleMotorID = 13;
            public static final int canCoderID = 17;
            public static final int RelativeEncoder1 = 23;
            public static final int RelativeEncoder2 = 18;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(291);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class ArmConstants{


        public static final int ArmMasterID = 59; //Alternate Encoder
        public static final int ArmFollowerID = 60;
        public static final int ArmExtenderID = 58;
        public static final int HandMotorID = 57;

        public static final int ArmPCMForward = 11;
        public static final int ArmPCMBackwards = 10;

        public static final int ArmAbsoluteActuator = 0;



        public static final double AbsoluteArmOffset = 65d;
  
    }

    public static final class HopConstants{   
        public static final int HopPCMForward = 9;
        public static final int HopPCMBackwards = 8;


    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }


 

    public static final Gains kArmGains = new Gains(0.000300, 0, 0.002, 0.000010/*156 */, 0, 1.0);
    public static final Gains kArmGains1 = new Gains(0.000200, 0, 0.002, 0.000010/*156 */, 0, 1.0);
    public static final Gains kArmGains2 = new Gains(0, 0, 0, 0.0/*156 */, 0, 1.0);
    public static final Gains kArmGains3 = new Gains(0.000050, 0, 0.002, 0.000010/*156 */, 0, 1.0);


    public static final Gains kArmExtendGains = new Gains(0.25, 0.0, 0, 0, 0, 1.0);

}

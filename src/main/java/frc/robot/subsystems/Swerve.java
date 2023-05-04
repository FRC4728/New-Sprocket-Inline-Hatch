package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {

  //  private PhotonVisionSubsystem vision1;
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveModule[] mSwerveMods;
    private final PIDController m_YController;
   // private DriverStation m_driverStation;
    private DriverStation.Alliance allianceColor;
    public Pigeon2 gyro;
   private double KpAim = -0.1;
    private double KpDistance = -0.1;
    private double min_aim_command = 0.05;

  // private static final edu.wpi.first.math.Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0, 0, 0);

  //  private static final edu.wpi.first.math.Vector<N3> driveMeasurementStdDevs = VecBuilder.fill(0, 0, 0);

    public Swerve() {//PhotonVisionSubsystem vision1) {
      //  this.vision1 = vision1; 
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();



         allianceColor = DriverStation.getAlliance();

         SmartDashboard.putString("Alliance Color", allianceColor.toString());


        m_YController = new PIDController(.007, 0, 0);

         m_YController.setTolerance(0.05);
  

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

       new Thread(() -> {
        try {
            //possible
        //    resetEncoders();
            Thread.sleep(3000);
            resetModulesToAbsolute();
            SmartDashboard.putBoolean("True", true);
        } catch (Exception e) {
        }
    }).start();
    
         poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions(), new Pose2d(0, 0, new Rotation2d(0)));//, driveMeasurementStdDevs);// visionMeasurementStdDevs);
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
        
    }

    public void drive(Translation2d translation, double rotation, boolean quickTurn, boolean zoom, boolean angletohop) {
      if (angletohop == false){  SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()));
                               
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], quickTurn, zoom, false);
        }}

        if (angletohop == true){ 

            if  (quickTurn == true) {
            if (allianceColor == Alliance.Red){
     double y_SetPoint = 90;
            
     double y_Speed =  m_YController.calculate(getYaw().getDegrees(), y_SetPoint);
            SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    y_Speed, 
                                    getYaw()));
                               
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], quickTurn, zoom, false);
        }}
        else if (allianceColor == Alliance.Blue){
            double y_SetPoint = -90;
                   
            double y_Speed =  m_YController.calculate(getYaw().getDegrees(), y_SetPoint);
                   SwerveModuleState[] swerveModuleStates =
                   Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                       ChassisSpeeds.fromFieldRelativeSpeeds(
                                           translation.getX(), 
                                           translation.getY(), 
                                           y_Speed, 
                                           getYaw()));
                                      
               SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
       
               for(SwerveModule mod : mSwerveMods){
                   mod.setDesiredState(swerveModuleStates[mod.moduleNumber], quickTurn, zoom, false);
               }} 
        }
            else if (quickTurn == false){
             if (allianceColor == Alliance.Red){
                    double y_SetPoint = 90;
                           
                    double y_Speed =  m_YController.calculate(getYaw().getDegrees(), y_SetPoint);
                           SwerveModuleState[] swerveModuleStates =
                           Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                               ChassisSpeeds.fromFieldRelativeSpeeds(
                                                   translation.getX(), 
                                                   translation.getY(), 
                                                   y_Speed, 
                                                   getYaw()));
                                              
                       SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
               
                       for(SwerveModule mod : mSwerveMods){
                           mod.setDesiredState(swerveModuleStates[mod.moduleNumber], quickTurn, zoom, false);
                       }}
             else if (allianceColor == Alliance.Blue){
                           double y_SetPoint = -90;
                                  
                           double y_Speed =  m_YController.calculate(getYaw().getDegrees(), y_SetPoint);
                                  SwerveModuleState[] swerveModuleStates =
                                  Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                                      ChassisSpeeds.fromFieldRelativeSpeeds(
                                                          translation.getX(), 
                                                          translation.getY(), 
                                                          y_Speed, 
                                                          getYaw()));
                                                     
                              SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
                      
                              for(SwerveModule mod : mSwerveMods){
                                  mod.setDesiredState(swerveModuleStates[mod.moduleNumber], quickTurn, zoom, false);
                              }} 
            }

            }
    }

    public void DriveAutomatically(){
        for(SwerveModule mod : mSwerveMods){
            mod.driveSlowly();
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false, false, true);
        }
    }    

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        double yeehaw = gyro.getYaw();
        return Rotation2d.fromDegrees(Math.IEEEremainder(yeehaw, 360));
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void updateOdometry() {
        poseEstimator.update(getYaw(), getModulePositions());  

}
    @Override
    public void periodic(){
        updateOdometry();
                    SmartDashboard.putString("Robot Pose", poseEstimator.getEstimatedPosition().toString());
    }


    public double getTheta() {
        return getYaw().getRadians();
    }
    public void stopModules() {
        for(SwerveModule mod : mSwerveMods){
            mod.stop();
        }
    }

    public double getPitch()
    {
        return gyro.getPitch();
    }
}

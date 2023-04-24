package frc.robot.Autos;

import frc.robot.Constants;
import frc.robot.commands.ArmCommands.ArmHighCommand;
import frc.robot.commands.ArmCommands.ArmHighCubeCommand;
import frc.robot.commands.ArmCommands.ArmHighHoldCommand;
import frc.robot.commands.ArmCommands.ArmStopCommand;
import frc.robot.commands.ArmCommands.ArmToGroundAuto;
import frc.robot.commands.ArmCommands.ArmToGroundCommand;
import frc.robot.commands.ArmCommands.ArmToHomeCommand;
import frc.robot.commands.ExtendCommands.ArmExtendCommand;
import frc.robot.commands.ExtendCommands.ArmRetractCommand;
import frc.robot.commands.ExtendCommands.ExtendToGroundCommand;
import frc.robot.commands.HandCommands.HandInCubeCommand;
import frc.robot.commands.HandCommands.HandOutConeCommand;
import frc.robot.commands.HandCommands.HandOutCubeCommand;
import frc.robot.commands.HandCommands.HandStopCommand;
import frc.robot.commands.HopCommands.HopperIn;
import frc.robot.commands.PistonCommands.ArmPistonExtendCommand;
import frc.robot.commands.PistonCommands.ArmPistonRetractCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtendingSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.PistonSubsystem;
import frc.robot.subsystems.Swerve;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class TwoBallTopAuto extends SequentialCommandGroup {

  public TwoBallTopAuto(Swerve s_Swerve, ArmSubsystem s_Arm, HandSubsystem s_Hand, ExtendingSubsystem s_Extend,
      HopperSubsystem s_Hopper, PistonSubsystem s_Piston) {
    // Path Planner Path
    String robot_path = "2 Ball Top";
    String end_of_path = "End of 2 Ball Top";
    PathPlannerTrajectory TestPath = PathPlanner.loadPath(robot_path, new PathConstraints(2.3, 2.3));
    PathPlannerTrajectory EndofTwoBall = PathPlanner.loadPath(end_of_path, new PathConstraints(2.3, 2.3));
    HashMap<String, Command> eventMap = new HashMap<>();
 
      eventMap.put("ArmHighConePlace", new SequentialCommandGroup(
        new ArmRetractCommand(s_Extend).until(() -> (s_Extend.getEncoderExtend() <= .7)),
        new ParallelCommandGroup(
          new ArmHighCommand(s_Arm).until(() ->(s_Arm.getEncoderActuate() > 117.5) &  (s_Arm.getEncoderActuate() < 118.5)),
          new ArmPistonExtendCommand(s_Piston).beforeStarting(new WaitUntilCommand(() -> s_Arm.getEncoderActuate() > 60)).withTimeout(2.2),
          new ArmExtendCommand(s_Extend).beforeStarting(new WaitUntilCommand(() -> s_Arm.getEncoderActuate() > 90)).until(() -> ( s_Extend.getEncoderExtend() < 62) &  (s_Extend.getEncoderExtend() > 58))),  
        new ParallelRaceGroup(
            new ArmHighHoldCommand(s_Arm),
            new ParallelRaceGroup(
                new HandOutConeCommand(s_Hand),
                new WaitCommand(.4)
            )
        )
       )
      );
      eventMap.put("ArmHighCubeRetract", new SequentialCommandGroup(
       // new ArmToHomeCommand(s_Arm).until(() -> (s_Arm.getEncoderActuate() > -7.5) & (s_Arm.getEncoderActuate() < -2.5)),
        //new ArmStopCommand(s_Arm).withTimeout(.05),
        new ParallelCommandGroup(  
          new ArmPistonRetractCommand(s_Piston).withTimeout(.7) ,
          new ExtendToGroundCommand(s_Extend).until(() -> (s_Extend.getEncoderExtend() < 48.7) & (s_Extend.getEncoderExtend() >47.7))
        ),    

        new ParallelCommandGroup(
          new SequentialCommandGroup(
            new ArmToGroundCommand(s_Arm).until(() -> (s_Arm.getEncoderActuate() < 34.9) & (s_Arm.getEncoderActuate() >34.4)),
            new ArmStopCommand(s_Arm).withTimeout(.1)),

        new HandInCubeCommand(s_Hand).until(() -> (s_Hand.getvoltageCube() == true))
        ),
        new ArmRetractCommand(s_Extend).until (() -> s_Extend.getEncoderExtend() < .7)
       
         )
        );


        eventMap.put("ArmUp", new SequentialCommandGroup(
            new ArmHighCubeCommand(s_Arm).until(() ->(s_Arm.getEncoderActuate() > 104.5) &  (s_Arm.getEncoderActuate() < 106.5)),
           new ParallelRaceGroup(
            new ArmHighHoldCommand(s_Arm),
            new ArmExtendCommand(s_Extend).until(() -> ( s_Extend.getEncoderExtend() < 62) &  (s_Extend.getEncoderExtend() > 58))
       )
       //           new ArmStopCommand(s_Arm).withTimeout(.1)

            )
           );


     eventMap.put("ArmHighCube", new SequentialCommandGroup(
        //new ArmRetractCommand(s_Extend).until(() -> (s_Extend.getEncoderExtend() <=.7)),
        new HandStopCommand(s_Hand).withTimeout(.1),
        new ArmHighCubeCommand(s_Arm).until(() ->(s_Arm.getEncoderActuate() > 104.5) &  (s_Arm.getEncoderActuate() < 106.5)),
        new ParallelRaceGroup(
             new ArmHighHoldCommand(s_Arm),
             new ArmExtendCommand(s_Extend).until(() -> ( s_Extend.getEncoderExtend() < 62) &  (s_Extend.getEncoderExtend() > 58))
        ),
        new ParallelRaceGroup(
            new ArmHighHoldCommand(s_Arm),
            new ParallelRaceGroup(
                    new HandOutCubeCommand(s_Hand),
                    new WaitCommand(.4)      
            )
    ),
    new HandStopCommand(s_Hand).withTimeout(.1)));
   



    
    // 4. Construct command to follow trajectory

    // auto builder can use events added in through Path Planner
    SwerveAutoBuilder autobuilder = new SwerveAutoBuilder(
        s_Swerve::getPose,
        s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.Swerve.swerveKinematics,
        new PIDConstants(1.5, 0, 0.15, .005),
        new PIDConstants(.9, 0, 0.05, .005),
        s_Swerve::setModuleStates,
        eventMap,
        true,
        s_Swerve);

    Command fullAuto = autobuilder.fullAuto(TestPath);
    Command fullAuto2 = autobuilder.fullAuto(EndofTwoBall);


    // 5. Add some init and wrap-up, and return everything
    addCommands(
        new SequentialCommandGroup(
            // new InstantCommand(() ->
            // swerveSubsystem.resetOdometry(TestPath.getInitialPose())),
            fullAuto,
            fullAuto2,
            new InstantCommand(() -> s_Swerve.stopModules())));

    // new InstantCommand(() -> swerveSubsystem.getPose()));
  }
} 
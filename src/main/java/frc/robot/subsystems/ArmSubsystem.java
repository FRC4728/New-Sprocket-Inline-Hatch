package frc.robot.subsystems;

import com.fasterxml.jackson.databind.ser.std.StdArraySerializers.FloatArraySerializer;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    // The motors on the left side of the drive.

    CANSparkMax m_ArmMaster = new CANSparkMax(Constants.ArmConstants.ArmMasterID, MotorType.kBrushless);
    CANSparkMax m_ArmFollower = new CANSparkMax(Constants.ArmConstants.ArmFollowerID, MotorType.kBrushless);

    public double kP, kI, kD, kFF, kArbFF;
    private PowerDistribution m_PDP;

    // private PowerDistribution m_PDP = new PowerDistribution(0, ModuleType.kCTRE);

    private SparkMaxPIDController m_PIDControllerActuate;

    private RelativeEncoder m_encoderActuate;

    private DutyCycleEncoder angleEncoder;

    public double maxVel, maxAcc;

    double processVariable;

    public ArmSubsystem(PowerDistribution funny) {

        m_PDP = funny;

        kP = 0.000050;
        kI = 0;
        kD = 0.002;
        kFF = 0.000000;

        angleEncoder = new DutyCycleEncoder(new DigitalInput(Constants.ArmConstants.ArmAbsoluteActuator));
        m_encoderActuate = m_ArmMaster.getAlternateEncoder(AlternateEncoderType.kQuadrature, 4096);

        resetEncoders();

        new Thread(() -> {
            try {
                Thread.sleep(1500);
                resetToAbsolute();

            } catch (Exception e) {
            }
        }).start();

        m_ArmMaster.set(0);
        m_ArmFollower.set(0);

        m_ArmMaster.restoreFactoryDefaults();
        m_ArmFollower.restoreFactoryDefaults();

        m_ArmMaster.setInverted(false);
        m_ArmFollower.setInverted(false);

        m_encoderActuate.setPositionConversionFactor(386.909091);

        m_ArmMaster.setIdleMode(IdleMode.kBrake);
        m_ArmFollower.setIdleMode(IdleMode.kBrake);

        m_PIDControllerActuate = m_ArmMaster.getPIDController();

        m_PIDControllerActuate.setFeedbackDevice(m_encoderActuate);

        // set PID coefficients
        // m_PIDControllerActuate.setP(kP);
        // m_PIDControllerActuate.setI(kI);
        // m_PIDControllerActuate.setD(kD);
        // m_PIDControllerActuate.setIZone(kIz);
        // m_PIDControllerActuate.setFF(kFF);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Arb Feed Forward", kArbFF);

        SmartDashboard.putNumber("Set Rotations", 0);
        // m_PIDControllerActuate.setP(Constants.kArmGains.kP, 0);
        // m_PIDControllerActuate.setI(Constants.kArmGains.kI, 0);
        // m_PIDControllerActuate.setD(Constants.kArmGains.kD, 0);
        // m_PIDControllerActuate.setFF(Constants.kArmGains.kF, 0);

        m_PIDControllerActuate.setP(Constants.kArmGains.kP, 0);
        m_PIDControllerActuate.setI(Constants.kArmGains.kI, 0);
        m_PIDControllerActuate.setD(Constants.kArmGains.kD, 0);
        m_PIDControllerActuate.setFF(Constants.kArmGains.kF, 0);

        m_PIDControllerActuate.setP(Constants.kArmGains1.kP, 1);
        m_PIDControllerActuate.setI(Constants.kArmGains1.kI, 1);
        m_PIDControllerActuate.setD(Constants.kArmGains1.kD, 1);
        m_PIDControllerActuate.setFF(Constants.kArmGains1.kF, 1);

        m_PIDControllerActuate.setP(Constants.kArmGains2.kP, 2);
        m_PIDControllerActuate.setI(Constants.kArmGains2.kI, 2);
        m_PIDControllerActuate.setD(Constants.kArmGains2.kD, 2);
        m_PIDControllerActuate.setFF(Constants.kArmGains2.kF, 2);

        m_PIDControllerActuate.setP(Constants.kArmGains3.kP, 3);
        m_PIDControllerActuate.setI(Constants.kArmGains3.kI, 3);
        m_PIDControllerActuate.setD(Constants.kArmGains3.kD, 3);
        m_PIDControllerActuate.setFF(Constants.kArmGains3.kF, 3);

        // maxVel = 5676;
        // maxAcc = 5676;

        maxVel = 5676;
        maxAcc = 2838;

        int smartMotionSlot = 0;

        m_ArmMaster.enableVoltageCompensation(12);
        m_PIDControllerActuate.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        m_PIDControllerActuate.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        m_PIDControllerActuate.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);

        m_PIDControllerActuate.setSmartMotionMaxVelocity(maxVel, 1);
        m_PIDControllerActuate.setSmartMotionMaxAccel(5676, 1);
        m_PIDControllerActuate.setSmartMotionAllowedClosedLoopError(.1, 1);

        m_PIDControllerActuate.setSmartMotionMaxVelocity(100, 2);
        m_PIDControllerActuate.setSmartMotionMaxAccel(100, 2);
        m_PIDControllerActuate.setSmartMotionAllowedClosedLoopError(.1, 2);

        m_PIDControllerActuate.setSmartMotionMaxVelocity(maxVel, 3);
        m_PIDControllerActuate.setSmartMotionMaxAccel(5676, 3);
        m_PIDControllerActuate.setSmartMotionAllowedClosedLoopError(.1, 3);

        m_ArmFollower.follow(m_ArmMaster, false);

        m_ArmMaster.setSmartCurrentLimit(11);
        m_ArmFollower.setSmartCurrentLimit(11);

        m_ArmMaster.burnFlash();
        m_ArmFollower.burnFlash();

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm position", m_encoderActuate.getPosition());

        // read PID coefficients from SmartDashboard
    //    double p = SmartDashboard.getNumber("P Gain", 0);
      //  double i = SmartDashboard.getNumber("I Gain", 0);
        //double d = SmartDashboard.getNumber("D Gain", 0);
       // double iz = SmartDashboard.getNumber("I Zone", 0);
       // double ff = SmartDashboard.getNumber("Feed Forward", 0);
        // double ArbFF = SmartDashboard.getNumber("Arb Feed Forward", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to
        // controller
        // if((p != kP)) { m_PIDControllerActuate.setP(p,3); kP = p; }
        // if((i != kI)) { m_PIDControllerActuate.setI(i,3); kI = i; }
        // if((d != kD)) { m_PIDControllerActuate.setD(d,3); kD = d; }
        // if((ff != kFF)) { m_PIDControllerActuate.setFF(ff,3); kFF = ff; }
        // if((ArbFF != kArbFF)) { kArbFF.set; kArbFF = ArbFF; }

    
//master
   //   SmartDashboard.putNumber("Arm 1 Amps", m_PDP.getCurrent(13));

//follower
   //   SmartDashboard.putNumber("Arm 2 Amps", m_PDP.getCurrent(1));
      
     // ArmAmp1 = m_PDP.getCurrent(13);
     // ArmAmp2 = m_PDP.getCurrent(1);
     // voltage = m_PDP.getCurrent(8);
     

     }
    
    public void ActuateUp() {
        m_PIDControllerActuate.setReference(118, CANSparkMax.ControlType.kSmartMotion, 0, .075, ArbFFUnits.kPercentOut);
    }

    public void ActuateUpCube() {
        m_PIDControllerActuate.setReference(105, CANSparkMax.ControlType.kSmartMotion, 0, .075, ArbFFUnits.kPercentOut);
    }

    public void ActuateUpHold() {
        m_PIDControllerActuate.setReference(110, CANSparkMax.ControlType.kSmartMotion, 2, .02, ArbFFUnits.kPercentOut);
    }

    public void ActuateMiddleHold() {
        m_PIDControllerActuate.setReference(105, CANSparkMax.ControlType.kSmartMotion, 2, .01, ArbFFUnits.kPercentOut);
    }

    public void ActuateMiddleCubeHold() {
        m_PIDControllerActuate.setReference(110, CANSparkMax.ControlType.kSmartMotion, 2, -.01, ArbFFUnits.kPercentOut);
    }

    public void Actuate(double Speed) {

        m_ArmFollower.set(Speed / 4);
        m_ArmMaster.set(Speed / 4);
    }

    public void ManualUp() {

        m_ArmFollower.set(.1);
        m_ArmMaster.set(.1);
    }

    public void ManualDown() {

        m_ArmFollower.set(-.1);
        m_ArmMaster.set(-.1);
    }

    public void ActuateToHopper() {
        m_PIDControllerActuate.setReference(-11, CANSparkMax.ControlType.kSmartMotion, 3, -.1, ArbFFUnits.kPercentOut);

    }

    public void ActuateGround() {

        m_PIDControllerActuate.setReference(33.5, CANSparkMax.ControlType.kSmartMotion, 0, .1, ArbFFUnits.kPercentOut);
        processVariable = m_encoderActuate.getPosition();

    }

    public void ActuateGroundAuto() {

        m_PIDControllerActuate.setReference(21.7, CANSparkMax.ControlType.kSmartMotion, 0, .1, ArbFFUnits.kPercentOut);
        processVariable = m_encoderActuate.getPosition();

    }

    public void ActuateHome() {

        m_PIDControllerActuate.setReference(-1, CANSparkMax.ControlType.kSmartMotion, 1, .1, ArbFFUnits.kPercentOut);
        processVariable = m_encoderActuate.getPosition();
    }

    public void ActuateHomeHold() {

        m_PIDControllerActuate.setReference(0, CANSparkMax.ControlType.kSmartMotion, 3, 0, ArbFFUnits.kPercentOut);
        processVariable = m_encoderActuate.getPosition();

    }

    public void ActuateMiddle() {
        m_PIDControllerActuate.setReference(110, CANSparkMax.ControlType.kSmartMotion, 0, .12, ArbFFUnits.kPercentOut);

    }

    public void ActuateMiddleCube() {
        m_PIDControllerActuate.setReference(90, CANSparkMax.ControlType.kSmartMotion, 0, 0, ArbFFUnits.kPercentOut);

    }

    public void Stop() {
        m_PIDControllerActuate.setReference(0, ControlType.kVelocity);
    }

    public void resetEncoders() {
        m_encoderActuate.setPosition(0);
    }

    public void resetToAbsolute() {
        double absolutePosition = (angleEncoder.getAbsolutePosition() * 386.909091)
                - Constants.ArmConstants.AbsoluteArmOffset;
        m_encoderActuate.setPosition(absolutePosition);
    }

    public double getEncoderActuate() {

        return m_encoderActuate.getPosition();
    }

}

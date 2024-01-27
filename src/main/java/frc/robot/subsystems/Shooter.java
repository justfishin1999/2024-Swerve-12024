package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {
    CANSparkMax m_topShooterMotor = new CANSparkMax(Constants.ShooterConstants.topShooterMotorID, MotorType.kBrushless);
    CANSparkMax m_bottomShooterMotor = new CANSparkMax(Constants.ShooterConstants.bottomShooterMotorID, MotorType.kBrushless);

    private SparkPIDController m_BottomShooterPIDController, m_TopShooterPidController;
    public double s_kP, s_kI, s_kD, s_kFF;
    public int velo, topVelo, bottomVelo;

    public Shooter() {
        s_kP = 0.0;
        s_kI = 0.0;
        s_kD = 0.0;
        s_kFF = 0.0;

        m_bottomShooterMotor.restoreFactoryDefaults();
        m_topShooterMotor.restoreFactoryDefaults();

        m_BottomShooterPIDController = m_bottomShooterMotor.getPIDController();
        m_TopShooterPidController = m_topShooterMotor.getPIDController();

        m_BottomShooterPIDController.setP(s_kP);
        m_TopShooterPidController.setP(s_kP);

        m_BottomShooterPIDController.setI(s_kI);
        m_TopShooterPidController.setI(s_kI);

        m_BottomShooterPIDController.setD(s_kD);
        m_TopShooterPidController.setD(s_kD);

        m_BottomShooterPIDController.setFF(s_kFF);
        m_TopShooterPidController.setFF(s_kFF);
    }

    public void periodic(){
        //Output the velocity of the shooter motors to the dashboard
        SmartDashboard.putNumber("Top Shooter Motor Velocity:",topVelo);
        SmartDashboard.putNumber("Bottom Shooter Motor Velocity:",bottomVelo);
    }

    public void shootSpeaker(int Velo){
        //set the velocity of the motor based on input value
        m_BottomShooterPIDController.setReference(Velo,CANSparkBase.ControlType.kVelocity);
        m_TopShooterPidController.setReference(Velo,CANSparkBase.ControlType.kVelocity);
    }

    public void shootAmp(int topVelo, int bottomVelo){
        //set the velocity of the motor based on input value
        m_BottomShooterPIDController.setReference(bottomVelo,CANSparkBase.ControlType.kVelocity);
        m_TopShooterPidController.setReference(topVelo,CANSparkBase.ControlType.kVelocity);
    }

    public void stop(){
        //stop the motor
        m_BottomShooterPIDController.setReference(0,CANSparkBase.ControlType.kVelocity);
        m_TopShooterPidController.setReference(0,CANSparkBase.ControlType.kVelocity);
    }
}

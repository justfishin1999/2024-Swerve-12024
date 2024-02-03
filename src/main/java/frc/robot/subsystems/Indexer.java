package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase{
    CANSparkMax m_topIndexMotor = new CANSparkMax(Constants.IndexerConstants.topIndexMotorID, MotorType.kBrushless);
    CANSparkMax m_bottomIndexMotor = new CANSparkMax(Constants.IndexerConstants.bottomIndexMotorID, MotorType.kBrushless);

    private SparkPIDController m_BottomIndexerPIDControlle, m_TopIndexerPIDControlle;
    public double s_kP, s_kI, s_kD, s_kFF;
    public int velo, topVelo, bottomVelo;

    public Indexer() {
        s_kP = Constants.IndexerConstants.index_P;
        s_kI = Constants.IndexerConstants.index_I;
        s_kD = Constants.IndexerConstants.index_D;
        s_kFF = Constants.IndexerConstants.index_FF;

        m_bottomIndexMotor.restoreFactoryDefaults();
        m_topIndexMotor.restoreFactoryDefaults();

        m_BottomIndexerPIDControlle = m_bottomIndexMotor.getPIDController();
        m_TopIndexerPIDControlle = m_topIndexMotor.getPIDController();

        m_BottomIndexerPIDControlle.setP(s_kP);
        m_TopIndexerPIDControlle.setP(s_kP);

        m_BottomIndexerPIDControlle.setI(s_kI);
        m_TopIndexerPIDControlle.setI(s_kI);

        m_BottomIndexerPIDControlle.setD(s_kD);
        m_TopIndexerPIDControlle.setD(s_kD);

        m_BottomIndexerPIDControlle.setFF(s_kFF);
        m_TopIndexerPIDControlle.setFF(s_kFF);
    }

    public void periodic(){
        //Output the velocity of the shooter motors to the dashboard
        SmartDashboard.putNumber("Top Indexer Motor Velocity:",topVelo);
        SmartDashboard.putNumber("Bottom Indexer Motor Velocity:",bottomVelo);
    }

    public void runIndex(int Velo){
        //set the velocity of the motor based on input value
        m_BottomIndexerPIDControlle.setReference(Velo,CANSparkBase.ControlType.kVelocity);
        m_TopIndexerPIDControlle.setReference(Velo,CANSparkBase.ControlType.kVelocity);
    }

    public void stop(){
        //stop the motor
        m_BottomIndexerPIDControlle.setReference(0,CANSparkBase.ControlType.kVelocity);
        m_TopIndexerPIDControlle.setReference(0,CANSparkBase.ControlType.kVelocity);
    }
}


package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootAmp extends Command{
    private Shooter s_Shooter;
    private int c_velocityTop, c_velocityBottom;


    public ShootAmp(Shooter s_Shooter, int c_velocityTop, int c_velocityBottom){
        this.s_Shooter = s_Shooter;
        this.c_velocityTop = c_velocityTop;
        this.c_velocityBottom = c_velocityBottom;
        addRequirements(s_Shooter);
    }
    
    public void execute(){
        //Set the shooter to the desired speed
        s_Shooter.shootAmp(c_velocityTop,c_velocityBottom);
    }
    public void isInterrupted(){
        //Stop shooter if command is interrupted
        s_Shooter.stop();
    }
    public void end(){
        //stop shooter when command ends
        s_Shooter.stop();
    }
    public boolean isFinished(){
        return false;
    }
}

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootSpeaker extends Command{
    private Shooter s_Shooter;
    private int c_velocity;

    public ShootSpeaker(Shooter s_Shooter, int c_velocity){
        this.s_Shooter = s_Shooter;
        this.c_velocity = c_velocity;
        addRequirements(s_Shooter);
    }
    
    public void execute(){
        //Set the shooter to the desired speed
        s_Shooter.shootSpeaker(c_velocity);
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

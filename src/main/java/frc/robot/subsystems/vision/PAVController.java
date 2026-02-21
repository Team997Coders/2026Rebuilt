package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Position angle velocity
public class PAVController {
    private double velocity = 0;
    private double targetAngle = 25;

    public PAVController()
    {

    }

    public void update(double distance)
    {
        SmartDashboard.putNumber("Pav distance", distance);
        if (distance < 2.3)
        {
            targetAngle = 25;
        }
        else if (distance < 2.66){
            if (targetAngle == 40)
            {
                targetAngle = 35;
            }
        }
        else if (distance >= 2.66 && distance <= 3.6)
        {
            targetAngle = 35;
        }
        else if (distance > 3.6 && distance < 4){
            if (targetAngle == 25)
            {
                targetAngle = 35;
            }
        }
        else 
        {
            targetAngle = 40;
        }
        SmartDashboard.putNumber("pav angle", targetAngle);
        setTargetVelocity(distance);
        SmartDashboard.putNumber("pav target velocity", velocity);
    }

    public void setTargetVelocity(double distance)
    {
        if (targetAngle == 25)
        {
            velocity = 7.0164 + 1.279 * distance;
        }
        else if (targetAngle == 35)
        {
            velocity = 7.8625 + .7724 * distance;
        }
        else //40
        {
            velocity = 7.3463 + .9521 * distance;
        }
    }

    public double getAngle()
    {
        return targetAngle;
    }

    public double getVelocity()
    {
        return velocity;
    }
}

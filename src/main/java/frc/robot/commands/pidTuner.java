package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class pidTuner extends ProfiledPIDController{

    private Double[] pidValues = new Double[3];
    private double tolerance = 0.0;
    private final String name;
    private boolean pidTuningStatus = true;

    public pidTuner(String name, double kp, double ki, double kd, TrapezoidProfile.Constraints constraints)
    {
        super(kp, ki, kd, constraints);
        pidValues[0] = kp;
        pidValues[1] = ki;
        pidValues[2] = kd;
        this.name = name;
        SmartDashboard.putNumberArray(name, pidValues);
    }

    @Override
    /**
    * This is the original problem I found with the original ProfiledPidController 
    * I assumed the tolerance would be
    *   While the pid controller is within this tolerance of the goal
    *   + or - the tolerance, it gives a window in which the pid.calculate(measuredValue) will return 0
    * So in this special pid controller it will just run an if statement on the pid.calculate
    * And return 0 when you are within the tolerance
    * Your welcome :)
    */
    public void setTolerance(double tolerance)
    {
        this.tolerance = tolerance;
    }

    @Override
    /**
     * Updated calculate function does all this things normal calculate does and more!
     * 2 changes:
     * first now has tolerance done properly
     * second - checks smart dashboard for updated pid values and resets pid for easy tuning
     * Just pull up the number array in elastic, turn on the publish values bit, change the numbers and click the go button
     * Now you can tune pid without re deploying the code every 5 seconds
     * -Note- you can turn off the smart dashboard checking by calling setPidTuningStatus(false)
     * for competition where computation and comunication time matter
     */
    public double calculate(double measurement)
    {
        if (pidTuningStatus)
        {
            checkDashboardPidValues();
        }

        double goal = this.getGoal().position;
        if (Math.abs(goal - measurement) < tolerance)
        {
            return 0.0;
        }
        else
        {
            return super.calculate(measurement);
        }
    }

    /**
     * @param status
     * Whether or not you want this checking smart dashboard periodically
     * (false for competition is a good idea)
     */
    public void setPidTuningStatus(Boolean status)
    {
        pidTuningStatus = status;
    }


    public void checkDashboardPidValues()
    {

    }
}

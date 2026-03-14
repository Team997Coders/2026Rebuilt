import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.Constants;

public class Lights {
    private final DigitalOutput dio0 = new DigitalOutput(Constants.LightsConstants.dio0Pin);
    private final DigitalOutput dio1 = new DigitalOutput(Constants.LightsConstants.dio1Pin);
    private final DigitalOutput dio2 = new DigitalOutput(Constants.LightsConstants.dio2Pin);

    private void setState(int state = 0) {
        private String strstate = Integer.toBinaryString(state);

        dio0.set(Integer.parseInt(strstate.charAt(0)));
        dio1.set(Integer.parseInt(strstate.charAt(1)));
        dio2.set(Integer.parseInt(strstate.charAt(2)));
    }

    public void status_idleRed() {
        setState(0);
    }
    public void status_idleBlue() {
        setState(1);
    }
    public void status_activeRed() {
        setState(2);
    }
    public void status_activeBlue() {
        setState(3);
    }
    public void status_targetSearch() {
        setState(4);
    }
    public void status_targetLocked() {
        setState(5);
    }
    public void status_shoot() {
        setState(6);
    }
    public void status_climbing() {
        setState(7);
    }
    public void status_climbed() {
        setState(8);
    }
    
}
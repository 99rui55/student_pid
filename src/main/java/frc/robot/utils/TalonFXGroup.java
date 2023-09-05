package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class TalonFXGroup extends TalonFX {
    private final TalonFX[] followers;

    public TalonFXGroup(int... ids) {
        super(ids[0]);
        
        followers = new TalonFX[ids.length - 1];
        for (int i = 0; i < ids.length - 1; i++) {
            followers[i] = new TalonFX(i);
            followers[i].follow(this);
        }
    }

    @Override
    public void setInverted(boolean invert) {
        super.setInverted(invert);
        for (TalonFX follower : followers) {
            follower.setInverted(invert);
        }
    }
}

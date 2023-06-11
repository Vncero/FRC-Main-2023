package frc.robot.util;

import java.util.LinkedHashSet;

// concept taken from FRC Team 6328 Mechanical Advantage
// provides periodic for sensor-based "subsystems"
public abstract class VirtualSubsystem {
    private static final LinkedHashSet<VirtualSubsystem> virtualSubsystems = new LinkedHashSet<>();
    public static void runAllPeriodic() {
        virtualSubsystems.forEach(VirtualSubsystem::virtualPeriodic);
    }

    protected VirtualSubsystem() {
        virtualSubsystems.add(this);
    }

    public abstract void virtualPeriodic();
}
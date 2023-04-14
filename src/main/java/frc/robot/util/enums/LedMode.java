package frc.robot.util.enums;

public enum LedMode {
    /**
     * Use the LED Mode set in the current pipeline
     */
    DEFAULT(0),
    /**
     * Force off
     */
    OFF(1),
    /**
     * Force blink
     */
    BLINK(2),
    /**
     * Force on
     */
    ON(3);
    
    private final int value;
    
    LedMode(int value) {
        this.value = value;
    }
    
    public int getValue() {
        return value;
    }
}
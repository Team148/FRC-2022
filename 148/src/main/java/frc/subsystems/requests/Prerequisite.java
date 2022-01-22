package frc.subsystems.requests;

/**
 * A state which must be met before a Request can be acted upon
 */
public abstract class Prerequisite {
    public abstract boolean met();
}

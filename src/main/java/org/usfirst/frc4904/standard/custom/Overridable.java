package org.usfirst.frc4904.standard.custom;

public interface Overridable {
    /**
     * Set whether this object is overridden.
     *
     * @param isOverridden Whether to override the object or not
     */
    void setOverridden(boolean isOverridden);

    /**
     * Get whether this object is overridden.
     *
     * @return Whether this object is overridden.
     */
    boolean isOverridden();

    /**
     * Get whether this object is NOT overridden. Useful for creating
     * BooleanSuppliers with Java 8 syntax (e.g. this::isNotOverridden).
     *
     * @return Whether this object is NOT overridden.
     */
    default boolean isNotOverridden() {
        return !isOverridden();
    }
}

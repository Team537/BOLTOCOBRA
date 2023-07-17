/* THIS CODE WAS ADOPTED FROM 
FRC TEAM 4607 C.I.S.
(link) 
https://github.com/FRC4607/Comp-Bot-2023/blob/Comp-Bot/src/main/java/frc/robot/lib/AccelerationLimiter.java
*/
package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;

/**
 * A class that limits the acceleration and deceleration of the input value.
 */
public class AccelerationLimiter {

    private final double m_accelerationLimit;
    private final double m_decelerationLimit;
    private double m_prevVal;
    private double m_prevTime;

    /**
     * Creates a new AccelerationLimiter with the given a acceleration and deceleration rate
     * limits and initial value.
     *
     * @param accelerationLimit The rate-of-change limit away from zero in units per
     *                          second. This is expected to be positive.
     * @param decelerationLimit The rate-of-change limit towards from zero in units
     *                          per second. This is expected to be positive.
     * @param initialValue      The initial value of the input.
     */
    public AccelerationLimiter(double accelerationLimit, double decelerationLimit, double initialValue) {
        m_accelerationLimit = accelerationLimit;
        m_decelerationLimit = decelerationLimit;
        m_prevVal = initialValue;
        m_prevTime = WPIUtilJNI.now() * 1e-6;
    }

    /**
     * Creates a new AccelerationLimiter with the given a acceleration and deceleration rate
     * limits.
     *
     * @param accelerationLimit The rate-of-change limit away from zero in units per
     *                          second. This is expected to be positive.
     * @param decelerationLimit The rate-of-change limit towards from zero in units
     *                          per second. This is expected to be positive.
     */
    public AccelerationLimiter(double accelerationLimit, double decelerationLimit) {
        this(accelerationLimit, decelerationLimit, 0);
    }

    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public double calculate(double input) {
        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - m_prevTime;

        if (m_prevVal < 0) {
            m_prevVal += MathUtil.clamp(
                    input - m_prevVal,
                    -m_accelerationLimit * elapsedTime,
                    m_decelerationLimit * elapsedTime);
        } else {
            m_prevVal += MathUtil.clamp(
                    input - m_prevVal,
                    -m_decelerationLimit * elapsedTime,
                    m_accelerationLimit * elapsedTime);
        }

        m_prevTime = currentTime;
        return m_prevVal;
    }

    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit
     * when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(double value) {
        m_prevVal = value;
        m_prevTime = WPIUtilJNI.now() * 1e-6;
    }
}
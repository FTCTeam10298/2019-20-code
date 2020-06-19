package org.firstinspires.ftc.teamcode

/**
 * Holds PID values.
 */
class KPID (var propo: Double, var integ: Double, var deriv: Double){

    override fun toString(): String {
        return "Proportional: $propo \nIntegral: $integ \nDerivative: $deriv"
    }
}

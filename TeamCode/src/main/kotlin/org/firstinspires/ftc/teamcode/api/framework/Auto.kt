package org.firstinspires.ftc.teamcode.api.framework

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.api.Robot
import org.firstinspires.ftc.teamcode.api.ToggleableGamepad
import org.firstinspires.ftc.teamcode.api.ToggleableGamepad.Mode.DEBOUNCE
import org.firstinspires.ftc.teamcode.api.ToggleableGamepad.Mode.TOGGLE
import java.util.*
import kotlin.math.abs

interface Auto {
    fun init(robot: Robot)

    fun run(robot: Robot)
}

abstract class MirrorableAuto(val mirror: Boolean) : Auto {
    // Returns this but negative if mirror is true
    // Used for red vs blue auto
    protected val Double.m get() = if (mirror) -this else this
    protected val Int.m get() = this.toDouble().m
}

abstract class Configurator : LinearOpMode() {
    // Default config variables
    private val variables = mutableMapOf<String, Configurable<*>>()
    private val delay = number("Delay Before Start", delta = 0.25)
    val mirror = toggle("On Red Side")

    abstract fun build(): Auto

    override fun runOpMode() {
        AutoTransition.setup(this, "TeleOp")
        val robot = Robot(this)

        // Configure the autonomous
        var idx = 0
        var editing = false
        val varList = variables.toList()
        val gamepad = ToggleableGamepad(gamepad1, rbMode = DEBOUNCE, lbMode = DEBOUNCE,
                aMode = TOGGLE)
        while (!isStopRequested && !gamepad1.start) {
            // Load in and display the option
            val (name, opt) = varList[idx]
            telemetry.addData("State", if (editing) "Editing" else "Selecting")
            telemetry.addData("Option", name)
            telemetry.addData("Value", opt.value)
            telemetry.update()

            // Change the active option or its value
            gamepad.update() // Lock in the newest values for the gamepad
            editing = gamepad.a.value
            if (!editing) {
                when {
                    gamepad.leftBumper.value -> idx--
                    gamepad.rightBumper.value -> idx++
                }
                idx = idx.coerceIn(0, varList.size - 1)
            } else when {
                gamepad.leftBumper.value -> opt.dec()
                gamepad.rightBumper.value -> opt.inc()
            }
        }
        val auto = build()

        // Init the auto and wait for start
        auto.init(robot)
        waitForStart(robot) || return

        // Delay Handling
        if (delay.value != 0.0)
            sleep(abs(delay.value * 1000).toLong())

        // Run the auto
        auto.run(robot)
        robot.stop() // Ensure the robot stops moving

        // When auto is done, init TeleOp
    }

    interface Configurable<T> {
        var value: T
        fun inc()
        fun dec()
    }

    protected fun toggle(name: String, default: Boolean = false): Configurable<Boolean> {
        val configurable = object : Configurable<Boolean> {
            override var value: Boolean = default
            override fun inc() {
                value = !value
            }

            override fun dec() {
                value = !value
            }
        }
        variables[name] = configurable
        return configurable
    }

    protected fun number(name: String, default: Double = 0.0, min: Double = 0.0,
                         max: Double = 100.0, delta: Double = 1.0): Configurable<Double> {
        val configurable = object : Configurable<Double> {
            override var value: Double = default
                set(value) {
                    field = value.coerceIn(min, max)
                }

            override fun inc() {
                value += delta
            }

            override fun dec() {
                value -= delta
            }
        }
        variables[name] = configurable
        return configurable
    }
}
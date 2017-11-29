import java.util.*
import java.io.*
import java.math.*

/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/
fun main(args: Array<String>) {
    val input = Scanner(System.`in`)
    val surfaceN = input.nextInt() // the number of points used to draw the surface of Mars.
    val points = (0 until surfaceN).map { Point(input.nextFloat(), input.nextFloat()) }
    val surface = Surface((1 until points.size).mapIndexed { idx, _ -> SurfaceSegment(points[idx - 1], points[idx]) })

    // game loop
    while (true) {
        val lander = Lander(
            pos = Point(input.nextFloat(), input.nextFloat()),
            vel = Point(input.nextFloat(), input.nextFloat()),
            fuel = input.nextFloat(),
            rotate = input.nextFloat(),
            power = input.nextFloat()
        )
        // rotate power. rotate is the desired rotation angle. power is the desired thrust power.
        println("-20 3")
    }
}

val gravitForce = Point(3.711f, Math.PI.toFloat())

fun Float.sqr() = this * this
fun Float.sqrt() = Math.sqrt(this.toDouble()).toFloat()
fun Float.cos() = Math.cos(this.toDouble()).toFloat()
fun Float.sin() = Math.sin(this.toDouble()).toFloat()
fun Float.times2PI() = this * 2f * Math.PI.toFloat()
fun Float.atan2(x: Float) = Math.atan2(this.toDouble(), x.toDouble()).toFloat()
fun Float.toDegrees() = (this * Math.PI / 180).toFloat()
fun Float.toRadians() = (this * 180 / Math.PI).toFloat()
fun Float.abs() = Math.abs(this)
fun Double.toRandomInt(max: Int) = (this * max).toInt()

class Point(
    val x: Float,
    val y: Float
) {
    fun add(point: Point) = Point(x + point.x, y + point.y)
    fun sub(point: Point) = Point(x - point.x, y - point.y)
    fun mag2() = x.sqr() + y.sqr()
    fun mag() = mag2().sqrt()
    fun distance2(point: Point) = sub(point).mag2()
    fun distance(point: Point) = distance2(point).sqrt()
    fun toPolar() = Point(mag(), y.atan2(x))
    fun toCartesian() = Point(x * y.cos(), x * y.sin())
}

class Action(
    val power: Int,
    val angle: Int
)

class Lander(
    val pos: Point,
    val vel: Point,
    val fuel: Float,
    val rotate: Float,
    val power: Float
) {
    private fun copy(pos: Point = this.pos, vel: Point = this.vel, fuel: Float = this.fuel, rotate: Float = this.rotate, power: Float = this.power) = Lander(pos, vel, fuel, rotate, power)
    /**
     * Return a copy of this lander with position, fuel updated.
     */
    fun update(time: Float) = copy(pos = Point(pos.x + vel.x * time, pos.y + vel.y * time), fuel = (fuel - power * time).coerceAtLeast(0f))

    /**
     * Return a copy of this lander with velocity, rotate and power updated by a thrust.
     */
    fun thrust(action: Action): Lander {
        val power = action.power.toFloat()
        val angle = (action.angle + rotate).coerceIn(-90f..90f)
        val acceleration = Point(power, angle.toRadians())
        val composed = acceleration.add(gravitForce).toCartesian()
        return copy(vel = composed, rotate = angle, power = power)
    }
}

class SurfaceSegment(
    val a: Point,
    val b: Point
) {
    val polar = b.sub(a).toPolar()
    fun inRangeX(point: Point) = a.x >= point.x && b.x <= point.x
    fun yForX(x: Float) = x * polar.y.sin() / polar.y.cos()
    fun isBellowLine(point: Point) = point.y < yForX(point.x)
    fun isAboveLine(point: Point) = point.y > yForX(point.x)
    fun isInLine(point: Point) = point.y == yForX(point.x)
    fun isPlane() = a.x == b.x
    fun isLanded(lander: Lander) = isPlane() && lander.pos.y == a.y && lander.vel.x.abs() <= 20 && lander.vel.y.abs() <= 40 && lander.rotate == 0f
    fun crash(lander: Lander): Boolean {
        return if (inRangeX(lander.pos)) {
            val y = yForX(lander.pos.x)
            if (y == lander.pos.y) {
                lander.vel.x > 20 || lander.vel.y > 40 || lander.rotate != 0f
            } else y < lander.pos.y
        } else {
            false
        }
    }
}

class Surface(
    val segments: List<SurfaceSegment>
) {
    val planeSegments: List<SurfaceSegment> = segments.filter { it.isPlane() }
}

const val MAX_X = 6999f
const val MIN_X = 0f

enum class LanderStatus { FLYING, LANDED, DEAD }
class State(
    val surface: Surface,
    val lander: Lander
) {
    var status: LanderStatus

    init {
        status = when (true) {
            isDead() -> LanderStatus.DEAD
            isLanded() -> LanderStatus.LANDED
            else -> LanderStatus.FLYING
        }
    }

    fun nextState(action: Action): State =
        State(surface, lander.thrust(action).update(1f))


    fun isDead() = checkOut() || surface.segments.map { it.crash(lander) }.reduce { acc, curr -> acc || curr }
    fun isLanded() = surface.planeSegments.map { it.isLanded(lander) }.reduce { acc, curr -> acc || curr }

    fun computeStates(actions: List<Action>): State {
        var current = this
        for (action in actions) {
            if (current.status == LanderStatus.FLYING) current = current.nextState(action) else break
        }
        return current
    }

    fun evaluateDistance(): Float =
        surface.planeSegments.map {
            when (true) {
                it.inRangeX(lander.pos) -> lander.pos.y - it.a.y
                lander.pos.x < it.a.x -> lander.pos.distance(it.a)
                else -> lander.pos.distance(it.b)
            }
        }.reduce { acc, curr -> if (acc < curr) acc else curr }


    fun evaluate(): Float = when (true) {
        isDead() -> Float.MIN_VALUE
        isLanded() -> Float.MAX_VALUE
        else -> evaluateDistance()
    }

    fun checkOut() = lander.pos.x > MAX_X || lander.pos.x < MIN_X
}

// GA

const val GENOME_SIZE = 6
const val POPULATION_SIZE = 100
const val GENERATIONS = 5

val possibleTilts = (-15..15).toList()
val possibleTiltsSize = possibleTilts.size
val possibleThrusts = (0..4).toList()
val possibleThrustsSize = possibleThrusts.size

class Gene(
    val a: Double = Math.random(),
    val b: Double = Math.random()
) {
    fun toAction() = Action(possibleTilts[a.toRandomInt(possibleTiltsSize)], possibleThrusts[b.toRandomInt(possibleThrustsSize)])

}

class Genome(
    val genes: Array<Gene> = Array(GENOME_SIZE) { Gene() }
) {
    fun toActions() = genes.map { it.toAction() }
}

class GenomeState(val initialState: State,val genome: Genome) {
    var result = 0f
    fun simulate() {
        result = initialState.computeStates(genome.toActions()).evaluate()
    }
}

class GARunner(initState: State) {

    var population = Array(POPULATION_SIZE) { GenomeState(initState, Genome()) }

    fun run() {


    }

}
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
    val points = (0 until surfaceN).map { _ -> Point(input.nextDouble(), input.nextDouble()) }
    val surface = Line(points)

    // game loop
    var count = 0
    var commands = emptyList<State>()
    while (true) {
        val X = input.nextDouble()
        val Y = input.nextDouble()
        val hSpeed = input.nextDouble() // the horizontal speed (in m/s), can be negative.
        val vSpeed = input.nextDouble() // the vertical speed (in m/s), can be negative.
        val fuel = input.nextInt() // the quantity of remaining fuel in liters.
        val rotate = input.nextInt() // the rotation angle in degrees (-90 to 90).
        val power = input.nextInt() // the thrust power (0 to 4).
        val state = State(
            fuel = fuel,
            angle = rotate,
            power = power,
            particule = particule(X, Y, speed(hSpeed, vSpeed)))

        // Write an action using println()
        // To debug: System.err.println("Debug messages...");
        if (count == 0)
            commands = GARunner(state, surface).run()

        commands[count].apply { println("${this.angle} ${this.power}") }
        count += 1


        // rotate power. rotate is the desired rotation angle. power is the desired thrust power.
    }
}

class GARunner(
    val initialState: State,
    val ground: Line
) {
    fun run(): List<State> {
        val chimp = findBestChimp(this::createMarsLander2FromGenome, this::marsLander2Fitness)
        return chimp.result.trajectory.drop(1)
    }

    fun createMarsLander2FromGenome(genome: Array<Gene>): Lander {
        var previousPower = initialState.power
        var previousAngle = initialState.angle

        val ret = mutableListOf<ControlCmd>()
        (1..genomeSize).forEach {
            val power = (previousPower + genome[it - 1].b.asIntInRange(-1..1)).coerceIn(0..4)
            val angle = (previousAngle + genome[it - 1].a.asIntInRange(-15..15)).coerceIn(-90..90)
            ret.add(ControlCmd(power, 0))
            previousPower = power
            previousAngle = angle
        }
        return Lander(initialState, ret, ground)
    }

    fun marsLander2Fitness(lander: Lander) = when (lander.flystate) {
        FlyState.Landed -> lander.trajectory.last().fuel.toDouble() * 10
        FlyState.Flying -> -lander.ground.landingZoneDistanceTo(lander.trajectory.last().position)
        FlyState.Crashed -> with(lander.trajectory.last()) {
            -lander.ground.landingZoneDistanceTo(position) - speed.xSpeed - speed.ySpeed
        }
    }
}


fun cos(v: Double) = Math.cos(v)
fun sin(v: Double) = Math.sin(v)
fun sqrt(v: Double) = Math.sqrt(v)
fun toDegrees(v: Double) = v * 180 / Math.PI
fun toRadians(v: Double) = v * Math.PI / 180
fun random() = Math.random()
fun debug(any: Any) = System.err.println(any.toString())
fun Double.asInt(max: Int) = (this * (max + 1)).toInt()
fun Double.asIntInRange(range: IntRange) = range.first + (this * (range.last - range.first + 1)).toInt()
/**
 * A point in a 2 dimensions cartesian system.
 * Point + Vector -> Point
 * Point - Point  -> Vector
 */
data class Point(val x: Double, val y: Double) {

    infix operator fun plus(vector: Vector) = Point(x + vector.dx, y + vector.dy)
    infix operator fun minus(point: Point) = Vector(x - point.x, y - point.y)

    fun distanceTo(point: Point) = (point - this).length
}

/**
 * Vector * Double -> Vector
 * Vector + Vector -> Vector
 */
data class Vector(val dx: Double, val dy: Double) {

    infix operator fun plus(vector: Vector) = Vector(dx + vector.dx, dy + vector.dy)
    infix operator fun times(times: Double) = Vector(dx * times, dy * times)

    fun rotate(angle: Angle) =
        Vector(
            dx * cos(angle.rad) - dy * sin(angle.rad),
            dx * sin(angle.rad) + dy * cos(angle.rad)
        )

    val length: Double
        get() = sqrt(dx * dx + dy * dy)
}

/**
 * An angle that internally uses radian but can easily convert to degrees.
 */
data class Angle(val rad: Double) {
    val deg: Double
        get() = toDegrees(rad)
}

/**
 * Extension on Int to create angle :  15.deg()
 */
fun Int.deg(): Angle = Angle(toRadians(this.toDouble()))


/**
 * A line is a List of point.
 * It is comparable to point:   Line > Point
 */
data class Line(val points: List<Point>) {
    val landingZone: Pair<Point, Point>
    val ldx: Double
    val ldy: Double
    val lx2y1: Double
    val ly2x1: Double
    val ldx2: Double
    val ldy2: Double
    val lsqrt: Double

    init {
        if (points.size < 2)
            error("Should have 2 points at minimum.")
        landingZone = points.withIndex().first { it.value.y == points[it.index + 1].y }.let { it.value to points[it.index + 1] }
        ldx = landingZone.second.x - landingZone.first.x
        ldy = landingZone.second.y - landingZone.first.y
        lx2y1 = landingZone.second.x * landingZone.first.y
        ly2x1 = landingZone.second.y * landingZone.first.x
        ldx2 = ldx * ldx
        ldy2 = ldy * ldy
        lsqrt = Math.sqrt(ldy2 + ldx2)

    }

    fun landingZoneDistanceTo(p: Point) =
        Math.abs(ldy * p.x - ldx * p.y + lx2y1 - ly2x1) / lsqrt


    infix operator fun compareTo(point: Point) = (getYforX(point.x) - point.y).toInt()

    fun isHorizontalAtX(x: Double) = getSegmentFor(x).let {
        it.first.y == it.second.y
    }

    private fun getSegmentFor(x: Double) =
        (1 until points.size).first {
            points[it - 1].x <= x && x <= points[it].x
        }.let {
            Pair(points[it - 1], points[it])
        }

    private fun getYforX(x: Double) = getSegmentFor(x).let {
        it.first.y + (x - it.first.x) * (it.second.y - it.first.y) / (it.second.x - it.first.x)
    }

}

data class Time(val sec: Double)

val Double.sec: Time
    get() = Time(this)


/**
 * Speed is internally using a vector, allowing to compose it. speed12 = speed1 + speed2
 */
data class Speed(val direction: Vector) {

    infix operator fun plus(speed: Speed) = Speed(direction + speed.direction)

    val xSpeed: Double
        get() = direction.dx

    val ySpeed: Double
        get() = direction.dy

    override fun toString() = "(${xSpeed.format(2)}, ${ySpeed.format(2)})"
}

fun speed(xSpeed: Double, ySpeed: Double) = Speed(Vector(xSpeed, ySpeed))


/**
 * Acceleration is internally using a vector, allowing to compose it. acc12 = acc1 + acc2
 * Acceleration * Time -> Speed
 */
data class Acceleration(val vector: Vector) {

    infix operator fun times(time: Time) = Speed(vector * time.sec)

    infix operator fun plus(acceleration: Acceleration) = Acceleration(vector + acceleration.vector)
}

fun acceleration(xAcc: Double, yAcc: Double) = Acceleration(Vector(xAcc, yAcc))


/**
 * Represent something with an initial point and speed and on which an acceleration can be applied.
 */
data class Particule(val position: Point, val speed: Speed) {

    fun accelerate(acceleration: Acceleration, time: Time): Particule {
        val newSpeed = speed + acceleration * time

        val newPosition = position +
            speed.direction * time.sec +
            acceleration.vector * time.sec * time.sec * 0.5

        return Particule(newPosition, newSpeed)
    }

    override fun toString() = " x=${position.x.format(2)} y=${position.y.format(2)} speed= $speed"
}

fun particule(x: Double, y: Double, s: Speed) = Particule(Point(x, y), s)

fun Double.format(digits: Int) = java.lang.String.format("%.${digits}f", this)

val GRAVITY = acceleration(0.0, -3.711)
val maxX = 6999
val minX = 0

/**
 * power : 0, 1, 2, 3, 4
 * angle : -90, -75, ... , 0, +15, +30, ..., +75, +90
 */
data class ControlCmd(val power: Int = 0, val angle: Int = 0)

data class State(val fuel: Int, val power: Int, val angle: Int, val particule: Particule) {
    val position: Point
        get() = particule.position

    val speed: Speed
        get() = particule.speed
}

enum class FlyState {
    Landed, Crashed, Flying
}

/**
 * Handle the physics (acceleration, speed, fuel, ...)
 */
class Lander(initState: State, val cmds: List<ControlCmd>, val ground: Line) {

    val trajectory = mutableListOf(initState)
    var flystate = FlyState.Flying

    init {
        computeTrajectory()
    }

    fun computeTrajectory() {
        for ((i, cmd) in cmds.withIndex()) {
            val nextState = trajectory[i].computeNextState(cmd)
            trajectory.add(nextState)

            if (evaluateOutside(nextState)) return
            if (evaluateHitTheGround(nextState)) return
            if (evaluateNoFuel(nextState)) return
        }
    }

    fun evaluateOutside(state: State): Boolean {
        if (state.position.x > maxX || state.position.x < minX) {
            flystate = FlyState.Crashed
            return true
        }
        return false
    }

    fun evaluateHitTheGround(nextState: State): Boolean {
        if (ground > nextState.position) {
            if (nextState.angle == 0
                && nextState.speed.ySpeed > -40
                && nextState.speed.xSpeed.abs() <= 20
                && ground.isHorizontalAtX(nextState.position.x))
                flystate = FlyState.Landed
            else
                flystate = FlyState.Crashed
            return true
        }
        return false
    }

    fun evaluateNoFuel(nextState: State): Boolean {
        if (nextState.fuel <= 0) {
            flystate = FlyState.Crashed
            return true
        }
        return false
    }

    fun State.computeNextState(cmd: ControlCmd, time: Time = 1.0.sec): State {

        val newAngle = angle + (cmd.angle - angle).coerceAtLeast(-15).coerceAtMost(15)
        val newPower = power + (cmd.power - power).coerceAtMost(1).coerceAtLeast(-1)

        val thrustAcceleration = Acceleration((Vector(0.0, 1.0) * newPower.toDouble()).rotate(newAngle.deg()))
        val acceleration = GRAVITY + thrustAcceleration
        val newParticule = particule.accelerate(acceleration, time)
        val newFuel = fuel - newPower

        return State(newFuel, newPower, newAngle, newParticule)
    }
}

fun Double.abs(): Double = Math.abs(this)

// GA

val log = false
val elitism = true
val generationsCount = 10
val populationSize = 30
val genomeSize = 1200

var uniformRate = .5
var mutationRate = .06
var selectionRatio = .4


data class Gene(val a: Double = random(), val b: Double = random()) {
    /**
     * Random int from 0 to max (inclusive)
     */
    fun aAsInt(max: Int) = (a * (max + 1)).toInt()
}

class GenomeAndResult<out State>(val genome: Array<Gene>, create: (Array<Gene>) -> State) {
    val result = create(genome)
}

fun <T> findBestChimp(create: (Array<Gene>) -> T, fitness: (T) -> Double): GenomeAndResult<T> {

    fun Array<GenomeAndResult<T>>.sortByFitness() =
        this.apply {
            sortBy { fitness(it.result) }
            reverse()
                .apply { if (log) debug(joinToString(separator = " ", transform = { g -> fitness(g.result).toInt().toString().padStart(5) })) }
        }

    val population = Array(populationSize, { GenomeAndResult(buildGenome(genomeSize), create) }).sortByFitness()

    val chimpsFewGenerationsLater = (1..generationsCount)
        .fold(population) { pop, _ ->
            buildNextGeneration(pop, create).sortByFitness()
        }

    return chimpsFewGenerationsLater.first()
}

fun buildGenome(size: Int) = Array(size, { Gene() })

fun <T> buildNextGeneration(population: Array<GenomeAndResult<T>>,
                            create: (Array<Gene>) -> T): Array<GenomeAndResult<T>> {

    val newPop = population.copyOf()

    val elitismOffset = if (elitism) 1 else 0

    (elitismOffset until population.size).forEach {
        val genome1 = select(population).genome
        val genome2 = select(population).genome
        val genome = crossover(genome1, genome2)

        mutate(genome)

        newPop[it] = GenomeAndResult(genome, create)
    }
    return newPop
}

fun <T> select(population: Array<T>): T {
    for ((i, chimp) in population.withIndex()) {
        if (random() <= selectionRatio * (population.size - i) / population.size) {
            return chimp
        }
    }
    return population.first()
}

fun crossover(genome1: Array<Gene>, genome2: Array<Gene>) =
    Array(genome1.size, {
        if (random() <= uniformRate)
            genome1.get(it)
        else
            genome2.get(it)
    })

fun mutate(genome: Array<Gene>) {
    for ((i, _) in genome.withIndex()) {
        if (random() <= mutationRate) {
            genome[i] = Gene()
        }
    }
}

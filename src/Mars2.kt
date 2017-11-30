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
    val points = (0 until surfaceN).map { Point(input.nextDouble(), input.nextDouble()) }
    val surface = Surface((1 until points.size).map { idx -> SurfaceSegment(points[idx - 1], points[idx]) })

    debug("${gravitForce.toCartesian()}")

    // game loop
    while (true) {
        val lander = Lander(
            pos = Point(input.nextDouble(), input.nextDouble()),
            vel = Point(input.nextDouble(), input.nextDouble()),
            fuel = input.nextInt(),
            rotate = input.nextInt(),
            power = input.nextInt()
        )
        // rotate power. rotate is the desired rotation angle. power is the desired thrust power.
        val gaRunner = GARunner(State(surface, lander))
        gaRunner.run()
    }
}

val gravitForce = Point(3.711, 270.0.toRadians())

fun Double.sqr() = this * this
fun Double.sqrt() = Math.sqrt(this)
fun Double.cos() = Math.cos(this)
fun Double.sin() = Math.sin(this)
fun Double.atan2(x: Double) = Math.atan2(this, x)
fun Double.toRadians() = this * Math.PI / 180
fun Double.toDegrees() = this * 180 / Math.PI
fun Double.abs() = Math.abs(this)
fun Double.toRandomInt(max: Int) = (this * max).toInt()

class Point(
    val x: Double,
    val y: Double
) {
    fun add(point: Point) = Point(x + point.x, y + point.y)
    fun sub(point: Point) = Point(x - point.x, y - point.y)
    fun mag2() = x.sqr() + y.sqr()
    fun mag() = mag2().sqrt()
    fun distance2(point: Point) = sub(point).mag2()
    fun distance(point: Point) = distance2(point).sqrt()
    fun toPolar() = Point(mag(), y.atan2(x))
    fun toCartesian() = Point(x * y.cos(), x * y.sin())
    override fun toString(): String = "($x, $y)"
}

class Action(
    val angle: Int,
    val power: Int
) {
    fun print() = println("$angle $power")
}

class Lander(
    val pos: Point,
    val vel: Point,
    val fuel: Int,
    val rotate: Int,
    val power: Int
) {
    private fun copy(pos: Point = this.pos, vel: Point = this.vel, fuel: Int = this.fuel, rotate: Int = this.rotate, power: Int = this.power) = Lander(pos, vel, fuel, rotate, power)
    /**
     * Return a copy of this lander with position, fuel updated.
     */
    fun update(time: Double) = copy(pos = Point(pos.x + vel.x * time, pos.y + vel.y * time), fuel = (fuel - power * time).toInt().coerceAtLeast(0))

    /**
     * Return a copy of this lander with velocity, rotate and power updated by a thrust.
     */
    fun thrust(action: Action): Lander {
        val newPower = power + (action.power - power).coerceIn(-1..1)
        val newRotate = rotate + (action.angle - rotate).coerceIn(-15..15)
        val acceleration = Point(newPower.toDouble(), (90.0 - newRotate).toRadians())
        val composed = acceleration.add(gravitForce).toCartesian()
        return copy(vel = composed, rotate = newRotate, power = newPower)
    }

    override fun toString(): String =
        "Pos: ${pos.x}, ${pos.y} || Fuel: $fuel || Vel: ${vel.x} ${vel.y}"
}

class SurfaceSegment(
    val a: Point,
    val b: Point
) {
    val polar = b.sub(a).toPolar()
    fun inRangeX(point: Point) = a.x >= point.x && b.x <= point.x
    fun yForX(x: Double) = x * polar.y.sin() / polar.y.cos()
    fun isBellowLine(point: Point) = point.y < yForX(point.x)
    fun isAboveLine(point: Point) = point.y > yForX(point.x)
    fun isInLine(point: Point) = point.y == yForX(point.x)
    fun isPlane() = a.y == b.y
    fun isLanded(lander: Lander) = isPlane() && lander.pos.y == a.y && lander.vel.x.abs() <= 20 && lander.vel.y.abs() <= 40 && lander.rotate == 0
    fun crash(lander: Lander): Boolean {
        return if (inRangeX(lander.pos)) {
            val y = yForX(lander.pos.x)
            if (y == lander.pos.y) {
                lander.vel.x.abs() > 20 || lander.vel.y.abs() > 40 || lander.rotate != 0
            } else y > lander.pos.y
        } else {
            false
        }
    }

    override fun toString(): String =
        "Surface from $a to $b"
}

class Surface(
    val segments: List<SurfaceSegment>
) {
    val planeSegments: List<SurfaceSegment> = segments.filter { it.isPlane() }

    init {
        debug("$planeSegments")
    }
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
            isLanded() -> {
                debug("LANDED")
                LanderStatus.LANDED
            }
            else -> LanderStatus.FLYING

        }
    }

    private fun nextState(action: Action): State =
        State(surface, lander.thrust(action).update(1.0))


    private fun isDead() = checkOut() || surface.segments.map { it.crash(lander) }.reduce { acc, curr -> acc || curr }
    private fun isLanded() = surface.planeSegments.map { it.isLanded(lander) }.reduce { acc, curr -> acc || curr }

    fun computeStates(actions: List<Action>): State {
        // debug("First state $lander")
        var current = this
        for (action in actions) {
            if (current.status == LanderStatus.FLYING) current = current.nextState(action) else break
        }
        // debug("Last state ${current.lander}")
        return current
    }

    private fun checkOut() = lander.pos.x > MAX_X || lander.pos.x < MIN_X
}

// GA

const val GENOME_SIZE = 1200
const val POPULATION_SIZE = 20
const val GENERATIONS = 10
const val ELITISM = true
const val SELECTION_RATE = .4
const val UNIFORM_RATE = .5
const val MUTATION_RATE = .06

val possibleAngles = (-15..15).toList()
val possibleAnglesSize = possibleAngles.size
val possibleThrusts = (0..4).toList()
val possibleThrustsSize = possibleThrusts.size

class Gene(
    private val a: Double = Math.random(),
    private val b: Double = Math.random()
) {
    fun toAction() = Action(possibleAngles[a.toRandomInt(possibleAnglesSize)], possibleThrusts[b.toRandomInt(possibleThrustsSize)])

}

class Genome(
    val genes: Array<Gene> = Array(GENOME_SIZE) { Gene() }
) {
    fun toActions() = genes.map { it.toAction() }

    private fun mutate(gene: Gene): Gene = if (Math.random() <= MUTATION_RATE) Gene() else gene

    fun crossover(partner: Genome): Genome {
        val newGenes = Array(GENOME_SIZE) {
            if (Math.random() <= UNIFORM_RATE) {
                mutate(this.genes[it])
            } else {
                mutate(partner.genes[it])
            }
        }
        return Genome(newGenes)
    }
}

class GenomeState(val initialState: State, val genome: Genome) {

    var fitness: Double? = null

    fun simulate(): GenomeState {
        val lastState = initialState.computeStates(genome.toActions())
        fitness = when (lastState.status) {
            LanderStatus.FLYING -> lastState.surface.planeSegments.first().a.y - lastState.lander.pos.y
            LanderStatus.DEAD -> with(lastState.lander) { lastState.surface.planeSegments.first().a.y - pos.y + vel.y + 40 }
            LanderStatus.LANDED -> lastState.lander.fuel.toDouble()
        }
        return this
    }

}

fun debug(msg: String) = System.err.println(msg)

class GARunner(private val initState: State) {

    var population = Array(POPULATION_SIZE) { GenomeState(initState, Genome()) }

    val elitismOffset = if (ELITISM) 0 else 1

    private fun select(population: List<GenomeState>): GenomeState {
        population.forEachIndexed { i, sample ->
            if (Math.random() <= SELECTION_RATE * (POPULATION_SIZE - i) / POPULATION_SIZE) {
                return sample
            }
        }
        return population.first()
    }

    fun run() {
        for (i in 0..GENERATIONS) {
            population.forEach { it.simulate() }
            debug(population.joinToString { genomeState ->  genomeState.fitness.toString() })
            val orderedByFitness = population.sortedBy { it.fitness }.reversed().drop(elitismOffset)
            val newPopulation = Array(POPULATION_SIZE) {
                val genome1 = select(orderedByFitness).genome
                val genome2 = select(orderedByFitness).genome
                GenomeState(initState, genome1.crossover(genome2))
            }
            population = newPopulation
        }
        population.forEach { it.simulate() }
        val best = population.sortedBy { it.fitness }.last()
        best.genome.genes.first().toAction().print()

    }

}
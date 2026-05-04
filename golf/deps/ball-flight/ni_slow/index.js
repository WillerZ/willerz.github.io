import {
  defaultAtmosphere,
  defaultBallProperties,
  defaultSurfaceCondition,
  defaultWind,
  defaultWorld,
  makeFlatTerrain,
  makeSimplexHeightModifierTerrain
} from "./defaults.js";
import { SimulationError } from "./errors.js";
import {
  createPeakHeightCheckpointInternal,
  createReferencePlaneCheckpointInternal,
  createSimulatorInternal,
  resumeFromFlightCheckpointInternal,
  simulateInternal
} from "./internal/simulator.js";
import { traceRegime } from "./regime.js";

/**
 * @typedef {import("../core/index.js").SurfaceClass} SurfaceClass
 * @typedef {import("../core/index.js").Vector3} Vector3
 * @typedef {import("../core/index.js").Point3} Point3
 * @typedef {import("../core/index.js").TranslationalVelocity} TranslationalVelocity
 * @typedef {import("../core/index.js").RotationalVelocity} RotationalVelocity
 * @typedef {import("../core/index.js").LaunchState} LaunchStateInput
 * @typedef {import("./defaults.js").BallProperties} BallProperties
 * @typedef {import("./defaults.js").Atmosphere} Atmosphere
 * @typedef {import("./defaults.js").Wind} Wind
 * @typedef {import("./defaults.js").SurfaceCondition} SurfaceCondition
 * @typedef {import("./defaults.js").SimplexHeightModifierOptions} SimplexHeightModifierOptions
 * @typedef {import("./regime.js").TraceRegime} TraceRegime
 * @typedef {{heightM: number, dhDx?: number, dhDz?: number, classification: SurfaceClass, condition: SurfaceCondition}} TerrainSample
 * @typedef {{sample(x: number, z: number): TerrainSample}} TerrainProvider
 * @typedef {{ball: BallProperties, air: Atmosphere, windModel: Wind, terrainModel: TerrainProvider}} World
 * @typedef {{ball?: BallProperties, air?: Atmosphere, windModel?: Wind, terrainModel: TerrainProvider}} WorldInput
 * @typedef {{timeSeconds: number, regime: TraceRegime, position: Point3, translational: TranslationalVelocity, rotational: RotationalVelocity}} TraceSample
 * @typedef {{point: Point3, timeSeconds: number, incomingDirection: Vector3}} FirstImpact
 * @typedef {{impact: FirstImpact, finalRestPoint: Point3, trace: TraceSample[]}} SimulationResult
 * @typedef {object} FlightCheckpoint Opaque in-memory token returned by checkpoint creation APIs.
 * @typedef {{readonly environment: World, simulate(launchState: LaunchStateInput): SimulationResult, createReferencePlaneCheckpoint(launchState: LaunchStateInput): FlightCheckpoint, createPeakHeightCheckpoint(launchState: LaunchStateInput): FlightCheckpoint, resumeFromFlightCheckpoint(checkpoint: FlightCheckpoint): SimulationResult}} Simulator
 */

/**
 * Creates a reusable `ni_slow` simulator bound to a normalized world object.
 *
 * @param {WorldInput} world
 * @returns {Simulator}
 */
export function createSimulator(world) {
  return createSimulatorInternal(world);
}

/**
 * Runs a one-shot `ni_slow` simulation.
 *
 * @param {LaunchStateInput} launchState
 * @param {WorldInput} world
 * @returns {SimulationResult}
 */
export function simulate(launchState, world) {
  return simulateInternal(launchState, world, {});
}

/**
 * Creates an opaque checkpoint at the descending ball-center `Y=0` crossing after launch.
 *
 * The checkpoint is in-memory only and may be resumed against worlds that keep the same
 * ball, atmosphere, and wind configuration while changing the terrain provider.
 *
 * @param {LaunchStateInput} launchState
 * @param {WorldInput} world
 * @returns {FlightCheckpoint}
 */
export function createReferencePlaneCheckpoint(launchState, world) {
  return createReferencePlaneCheckpointInternal(launchState, world, {});
}

/**
 * Creates an opaque checkpoint at the first peak-height point before source terrain contact.
 *
 * The checkpoint is in-memory only and may be resumed against worlds that keep the same
 * ball, atmosphere, and wind configuration while changing the terrain provider.
 *
 * @param {LaunchStateInput} launchState
 * @param {WorldInput} world
 * @returns {FlightCheckpoint}
 */
export function createPeakHeightCheckpoint(launchState, world) {
  return createPeakHeightCheckpointInternal(launchState, world, {});
}

/**
 * Resumes a simulation from an opaque flight checkpoint.
 *
 * The resume terrain must not be above `checkpoint.position.y - ball.radiusM`
 * at the checkpoint. Lower terrain continues airborne flight until contact;
 * equal-height terrain impacts immediately.
 *
 * @param {FlightCheckpoint} checkpoint
 * @param {WorldInput} world
 * @returns {SimulationResult}
 */
export function resumeFromFlightCheckpoint(checkpoint, world) {
  return resumeFromFlightCheckpointInternal(checkpoint, world, {});
}

export {
  SimulationError,
  defaultAtmosphere,
  defaultBallProperties,
  defaultSurfaceCondition,
  defaultWind,
  defaultWorld,
  makeFlatTerrain,
  makeSimplexHeightModifierTerrain
};
export { traceRegime };

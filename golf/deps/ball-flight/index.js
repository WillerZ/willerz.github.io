/**
 * @typedef {import("./ni_slow/index.js").BallProperties} BallProperties
 * @typedef {import("./ni_slow/index.js").Atmosphere} Atmosphere
 * @typedef {import("./ni_slow/index.js").Wind} Wind
 * @typedef {import("./ni_slow/index.js").SurfaceCondition} SurfaceCondition
 * @typedef {import("./ni_slow/index.js").SimplexHeightModifierOptions} SimplexHeightModifierOptions
 * @typedef {import("./ni_slow/index.js").TerrainSample} TerrainSample
 * @typedef {import("./ni_slow/index.js").TerrainProvider} TerrainProvider
 * @typedef {import("./ni_slow/index.js").World} World
 * @typedef {import("./ni_slow/index.js").WorldInput} WorldInput
 * @typedef {import("./ni_slow/index.js").TraceRegime} TraceRegime
 * @typedef {import("./ni_slow/index.js").TraceSample} TraceSample
 * @typedef {import("./ni_slow/index.js").FirstImpact} FirstImpact
 * @typedef {import("./ni_slow/index.js").SimulationResult} SimulationResult
 * @typedef {import("./ni_slow/index.js").FlightCheckpoint} FlightCheckpoint
 * @typedef {import("./ni_slow/index.js").Simulator} Simulator
 */

export * from "./core/index.js";
export {
  SimulationError,
  createPeakHeightCheckpoint,
  createReferencePlaneCheckpoint,
  createSimulator,
  defaultAtmosphere,
  defaultBallProperties,
  defaultSurfaceCondition,
  defaultWind,
  defaultWorld,
  makeFlatTerrain,
  makeSimplexHeightModifierTerrain,
  resumeFromFlightCheckpoint,
  simulate,
  traceRegime
} from "./ni_slow/index.js";

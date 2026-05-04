import {
  addVector,
  cartesianToTranslationalVelocity,
  clamp,
  cross,
  dot,
  isFiniteVector3,
  isSurfaceClass,
  norm,
  normalizeUnitVector,
  rightHandRuleToRotationalVelocity,
  rotationalVelocityToRightHandRule,
  scaleVector,
  subtractVector,
  surfaceClass,
  translationalVelocityToCartesian,
  tryNormalizeUnitVector
} from "../../core/index.js";
import {
  defaultAtmosphere,
  defaultBallProperties,
  defaultSurfaceCondition,
  defaultWind
} from "../defaults.js";
import { SimulationError } from "../errors.js";
import { traceRegime } from "../regime.js";

/**
 * @typedef {import("../../core/index.js").Vector3} Vector3
 * @typedef {import("../../core/index.js").Point3} Point3
 * @typedef {import("../../core/index.js").SurfaceClass} SurfaceClass
 * @typedef {import("../defaults.js").BallProperties} BallProperties
 * @typedef {import("../defaults.js").Atmosphere} Atmosphere
 * @typedef {import("../defaults.js").Wind} Wind
 * @typedef {import("../defaults.js").SurfaceCondition} SurfaceCondition
 * @typedef {import("../index.js").FirstImpact} FirstImpact
 * @typedef {import("../index.js").LaunchStateInput} LaunchStateInput
 * @typedef {import("../index.js").FlightCheckpoint} FlightCheckpoint
 * @typedef {import("../index.js").SimulationResult} SimulationResult
 * @typedef {import("../index.js").Simulator} Simulator
 * @typedef {import("../index.js").TerrainProvider} TerrainProvider
 * @typedef {import("../index.js").TraceSample} TraceSample
 * @typedef {import("../index.js").World} World
 * @typedef {import("../index.js").WorldInput} WorldInput
 * @typedef {"flight" | "hop" | "skid" | "roll_with_slip" | "pure_roll" | "rest"} Mode
 * @typedef {{disableAerodynamics?: boolean, disableSpinDecay?: boolean}} SimulationOverrides
 * @typedef {{position: Point3, velocity: Vector3, omega: Vector3}} InternalState
 * @typedef {{eta: Vector3, rngState: bigint, spare: number | null}} GustState
 * @typedef {{environment: World, constants: {pressurePascals: number, densityKgPerM3: number, viscosityPaS: number}, state: InternalState, timeSeconds: number, airStep: number, currentMode: Mode, gustState: GustState, trace: TraceSample[], firstImpact: FirstImpact | null, bounceTerminationCounter: number}} SimulationContext
 * @typedef {"reference_plane" | "peak_height"} CheckpointKind
 * @typedef {{kind: CheckpointKind, environment: World, state: InternalState, timeSeconds: number, airStep: number, gustState: GustState}} CheckpointPayload
 * @typedef {{enforceLaunchClearance?: boolean}} WorldNormalizationOptions
 */

const gravity = 9.80665;
const gravityVector = { x: 0, y: -gravity, z: 0 };
const groundStepSeconds = 1e-3;
const airStepMinimum = 1e-6;
const airStepMaximum = 5e-3;
const relativeTolerance = 1e-8;
const positionAbsTolerance = 1e-7;
const velocityAbsTolerance = 1e-7;
const angularVelocityAbsTolerance = 1e-5;
const maxAirSteps = 500000;
const maxGroundSteps = 5000000;
const maxRootIterations = 80;
const rootTimeTolerance = 1e-6;
const rootPhiTolerance = 1e-6;
const nearPureRollSlipTolerance = 1e-2;

class FlightCheckpointToken {}

/** @type {WeakMap<FlightCheckpoint, CheckpointPayload>} */
const flightCheckpointPayloads = new WeakMap();

/** @type {{flight: Mode, hop: Mode, skid: Mode, rollWithSlip: Mode, pureRoll: Mode, rest: Mode}} */
export const modes = Object.freeze({
  flight: "flight",
  hop: "hop",
  skid: "skid",
  rollWithSlip: "roll_with_slip",
  pureRoll: "pure_roll",
  rest: "rest"
});

const turfLikeSurfaceClasses = new Set([
  surfaceClass.green,
  surfaceClass.fairway,
  surfaceClass.firstCut,
  surfaceClass.rough,
  surfaceClass.wetTurf,
  surfaceClass.ukGreen,
  surfaceClass.ukFairway
]);

const surfaceRows = new Map([
  [surfaceClass.green, { eBase: 0.24, muKBase: 0.22, cRrBase: 0.075, eMin: 0.10, eMax: 0.30, tiltMaxDeg: 4.0 }],
  [surfaceClass.fairway, { eBase: 0.27, muKBase: 0.40, cRrBase: 0.050, eMin: 0.15, eMax: 0.35, tiltMaxDeg: 3.0 }],
  [surfaceClass.firstCut, { eBase: 0.22, muKBase: 0.48, cRrBase: 0.085, eMin: 0.12, eMax: 0.30, tiltMaxDeg: 2.5 }],
  [surfaceClass.rough, { eBase: 0.18, muKBase: 0.55, cRrBase: 0.140, eMin: 0.08, eMax: 0.25, tiltMaxDeg: 2.0 }],
  [surfaceClass.bareSoil, { eBase: 0.58, muKBase: 0.45, cRrBase: 0.025, eMin: 0.35, eMax: 0.70, tiltMaxDeg: 0.0 }],
  [surfaceClass.wetTurf, { eBase: 0.10, muKBase: 0.30, cRrBase: 0.160, eMin: 0.05, eMax: 0.15, tiltMaxDeg: 1.0 }],
  [surfaceClass.sand, { eBase: 0.03, muKBase: 0.60, cRrBase: 0.350, eMin: 0.00, eMax: 0.10, tiltMaxDeg: 0.0 }],
  [surfaceClass.cartPath, { eBase: 0.72, muKBase: 0.55, cRrBase: 0.010, eMin: 0.60, eMax: 0.85, tiltMaxDeg: 0.0 }],
  [surfaceClass.ukGreen, { eBase: 0.24, muKBase: 0.22, cRrBase: 0.05283018867924528, eMin: 0.02, eMax: 0.30, tiltMaxDeg: 4.0 }],
  [surfaceClass.ukFairway, { eBase: 0.27, muKBase: 0.40, cRrBase: 0.20032223159191653, eMin: 0.03, eMax: 0.35, tiltMaxDeg: 3.0 }]
]);

function assertFiniteNumber(value, description) {
  if (!Number.isFinite(value)) {
    throw new TypeError(`Expected ${description} to be a finite number.`);
  }
  return value;
}

function cloneVector(value) {
  return { x: value.x, y: value.y, z: value.z };
}

function clonePoint(value) {
  return { x: value.x, y: value.y, z: value.z };
}

function cloneState(value) {
  return makeState(clonePoint(value.position), cloneVector(value.velocity), cloneVector(value.omega));
}

function normalizeBallProperties(input = defaultBallProperties()) {
  if (typeof input !== "object" || input === null) {
    throw new TypeError("Expected ball properties to be an object.");
  }

  const ball = {
    massKg: assertFiniteNumber(input.massKg, "ball.massKg"),
    diameterM: assertFiniteNumber(input.diameterM, "ball.diameterM"),
    radiusM: assertFiniteNumber(input.radiusM, "ball.radiusM"),
    areaM2: assertFiniteNumber(input.areaM2, "ball.areaM2"),
    inertiaKgM2: assertFiniteNumber(input.inertiaKgM2, "ball.inertiaKgM2"),
    aerodynamicProfile: String(input.aerodynamicProfile ?? defaultBallProperties().aerodynamicProfile)
  };

  if (ball.massKg <= 0 || ball.diameterM <= 0 || ball.radiusM <= 0 || ball.areaM2 <= 0 || ball.inertiaKgM2 <= 0) {
    throw new TypeError("Ball properties must be positive.");
  }

  return ball;
}

function normalizeAtmosphere(input = defaultAtmosphere()) {
  if (typeof input !== "object" || input === null) {
    throw new TypeError("Expected atmosphere to be an object.");
  }

  const output = {
    temperatureKelvin: assertFiniteNumber(input.temperatureKelvin, "air.temperatureKelvin"),
    relativeHumidity: assertFiniteNumber(input.relativeHumidity, "air.relativeHumidity")
  };

  if ("pressurePascals" in input && input.pressurePascals !== undefined) {
    output.pressurePascals = assertFiniteNumber(input.pressurePascals, "air.pressurePascals");
  }
  if ("altitudeMetres" in input && input.altitudeMetres !== undefined) {
    output.altitudeMetres = assertFiniteNumber(input.altitudeMetres, "air.altitudeMetres");
  }
  if (output.temperatureKelvin <= 0) {
    throw new TypeError("Air temperature must be positive.");
  }
  if (output.relativeHumidity < 0 || output.relativeHumidity > 1) {
    throw new TypeError("Relative humidity must be within [0, 1].");
  }

  return output;
}

function normalizeWind(input = defaultWind()) {
  if (typeof input !== "object" || input === null) {
    throw new TypeError("Expected windModel to be an object.");
  }

  const referenceWindMps = cloneVector(input.referenceWindMps ?? { x: 0, y: 0, z: 0 });
  const gustSigmaMps = cloneVector(input.gustSigmaMps ?? { x: 0, y: 0, z: 0 });
  if (!isFiniteVector3(referenceWindMps) || !isFiniteVector3(gustSigmaMps)) {
    throw new TypeError("Wind vectors must be finite.");
  }

  const output = {
    referenceWindMps,
    verticalMeanWindMps: assertFiniteNumber(input.verticalMeanWindMps ?? 0, "windModel.verticalMeanWindMps"),
    referenceHeightM: assertFiniteNumber(input.referenceHeightM ?? 10, "windModel.referenceHeightM"),
    roughnessLengthM: assertFiniteNumber(input.roughnessLengthM ?? 0.03, "windModel.roughnessLengthM"),
    zeroPlaneDisplacementM: assertFiniteNumber(input.zeroPlaneDisplacementM ?? 0, "windModel.zeroPlaneDisplacementM"),
    gustCorrelationTimeS: assertFiniteNumber(input.gustCorrelationTimeS ?? 1.5, "windModel.gustCorrelationTimeS"),
    gustSigmaMps,
    gustSeed: typeof input.gustSeed === "bigint" ? input.gustSeed : BigInt(Number(input.gustSeed ?? 0))
  };

  if (output.referenceHeightM <= 0 || output.roughnessLengthM <= 0 || output.gustCorrelationTimeS <= 0) {
    throw new TypeError("Wind referenceHeightM, roughnessLengthM, and gustCorrelationTimeS must be positive.");
  }

  return output;
}

function normalizeSurfaceCondition(input, classification) {
  const base = input ?? defaultSurfaceCondition(classification);
  if (typeof base !== "object" || base === null) {
    throw new TypeError("Expected surface condition to be an object.");
  }

  return {
    firmness: clamp(assertFiniteNumber(base.firmness, "surfaceCondition.firmness"), 0, 1),
    wetness: clamp(assertFiniteNumber(base.wetness, "surfaceCondition.wetness"), 0, 1),
    grassLengthIndex: clamp(assertFiniteNumber(base.grassLengthIndex, "surfaceCondition.grassLengthIndex"), 0, 1)
  };
}

function normalizeTerrainSample(sample) {
  if (typeof sample !== "object" || sample === null) {
    throw new TypeError("Terrain sample must be an object.");
  }
  const classification = sample.classification ?? surfaceClass.fairway;
  if (!isSurfaceClass(classification)) {
    throw new TypeError(`Unsupported surface classification: ${String(classification)}`);
  }
  const heightM = assertFiniteNumber(sample.heightM, "terrainSample.heightM");
  const dhDx = sample.dhDx === undefined ? undefined : assertFiniteNumber(sample.dhDx, "terrainSample.dhDx");
  const dhDz = sample.dhDz === undefined ? undefined : assertFiniteNumber(sample.dhDz, "terrainSample.dhDz");
  const condition = normalizeSurfaceCondition(sample.condition, classification);
  return { heightM, dhDx, dhDz, classification, condition };
}

/**
 * @param {any} input
 * @param {BallProperties} ball
 * @param {WorldNormalizationOptions} [options]
 */
function normalizeTerrainProvider(input, ball, options = {}) {
  if (typeof input !== "object" || input === null || typeof input.sample !== "function") {
    throw new TypeError("World terrainModel must expose a sample(x, z) function.");
  }

  if (options.enforceLaunchClearance !== false) {
    const origin = normalizeTerrainSample(input.sample(0, 0));
    if (origin.heightM > -ball.radiusM) {
      throw new TypeError("Terrain at launch is above the ball.");
    }
  }

  return {
    sample(x, z) {
      return normalizeTerrainSample(input.sample(x, z));
    }
  };
}

/**
 * @param {any} input
 * @param {WorldNormalizationOptions} [options]
 */
export function normalizeWorld(input, options = {}) {
  if (typeof input !== "object" || input === null) {
    throw new TypeError("Expected world to be an object.");
  }

  const ball = normalizeBallProperties(input.ball ?? defaultBallProperties());
  const air = normalizeAtmosphere(input.air ?? defaultAtmosphere());
  const windModel = normalizeWind(input.windModel ?? defaultWind());
  const terrainModel = normalizeTerrainProvider(input.terrainModel, ball, options);

  return { ball, air, windModel, terrainModel };
}

function normalizeLaunchState(input) {
  if (typeof input !== "object" || input === null) {
    throw new TypeError("Expected launchState to be an object.");
  }

  const translational = {
    speed: assertFiniteNumber(input.translational?.speed, "launchState.translational.speed"),
    direction: normalizeUnitVector(input.translational?.direction, "launchState.translational.direction")
  };
  const rotational = {
    angularSpeed: assertFiniteNumber(input.rotational?.angularSpeed, "launchState.rotational.angularSpeed"),
    axis: normalizeUnitVector(input.rotational?.axis, "launchState.rotational.axis")
  };

  if (translational.speed < 0 || rotational.angularSpeed < 0) {
    throw new TypeError("Launch speeds must be non-negative.");
  }

  return { translational, rotational };
}

function resolvePressurePascals(air) {
  if (air.pressurePascals !== undefined) {
    return air.pressurePascals;
  }
  if (air.altitudeMetres !== undefined) {
    const p0 = 101325;
    const t0 = 288.15;
    const lapse = 0.0065;
    const scale = 1 - lapse * air.altitudeMetres / t0;
    return p0 * Math.pow(scale, 5.255877);
  }
  return 101325;
}

function saturationVaporPressureCipm(temperatureKelvin) {
  return Math.exp(
    1.2378847e-5 * temperatureKelvin * temperatureKelvin -
      1.9121316e-2 * temperatureKelvin +
      33.93711047 -
      6.3431645e3 / temperatureKelvin
  );
}

function densityCipm2007(temperatureKelvin, pressurePascals, relativeHumidity) {
  const molarMassDryAir = 28.96546e-3;
  const molarMassWater = 18.01528e-3;
  const universalGasConstant = 8.314472;
  const temperatureCelsius = temperatureKelvin - 273.15;
  const pSv = saturationVaporPressureCipm(temperatureKelvin);
  const enhancementFactor =
    1.00062 + 3.14e-8 * pressurePascals + 5.6e-7 * temperatureCelsius * temperatureCelsius;
  const xV = clamp(relativeHumidity, 0, 1) * enhancementFactor * pSv / pressurePascals;

  const a0 = 1.58123e-6;
  const a1 = -2.9331e-8;
  const a2 = 1.1043e-10;
  const b0 = 5.707e-6;
  const b1 = -2.051e-8;
  const c0 = 1.9898e-4;
  const c1 = -2.376e-6;
  const d = 1.83e-11;
  const e = -0.765e-8;

  const compressibility =
    1 -
    (pressurePascals / temperatureKelvin) *
      (a0 +
        a1 * temperatureCelsius +
        a2 * temperatureCelsius * temperatureCelsius +
        (b0 + b1 * temperatureCelsius) * xV +
        (c0 + c1 * temperatureCelsius) * xV * xV) +
    (pressurePascals * pressurePascals) /
      (temperatureKelvin * temperatureKelvin) *
      (d + e * xV * xV);

  return (
    (pressurePascals * molarMassDryAir) /
    (compressibility * universalGasConstant * temperatureKelvin) *
    (1 - xV * (1 - molarMassWater / molarMassDryAir))
  );
}

function viscositySutherland(temperatureKelvin) {
  return 1.458e-6 * Math.pow(temperatureKelvin, 1.5) / (temperatureKelvin + 110.4);
}

function evaluateEnvironmentConstants(air) {
  const pressurePascals = resolvePressurePascals(air);
  if (!(pressurePascals > 0)) {
    throw new TypeError("Atmospheric pressure must be positive.");
  }
  const densityKgPerM3 = densityCipm2007(air.temperatureKelvin, pressurePascals, air.relativeHumidity);
  const viscosityPaS = viscositySutherland(air.temperatureKelvin);
  if (!(densityKgPerM3 > 1e-12) || !(viscosityPaS > 0)) {
    throw new TypeError("Computed air properties are invalid.");
  }
  return { pressurePascals, densityKgPerM3, viscosityPaS };
}

export function evaluateTerrain(terrainModel, xM, zM) {
  const delta = 0.01;
  const center = terrainModel.sample(xM, zM);
  const dhDx = center.dhDx ?? (terrainModel.sample(xM + delta, zM).heightM - terrainModel.sample(xM - delta, zM).heightM) / (2 * delta);
  const dhDz = center.dhDz ?? (terrainModel.sample(xM, zM + delta).heightM - terrainModel.sample(xM, zM - delta).heightM) / (2 * delta);
  const normal = tryNormalizeUnitVector({ x: -dhDx, y: 1, z: -dhDz }) ?? { x: 0, y: 1, z: 0 };
  return {
    heightM: center.heightM,
    dhDx,
    dhDz,
    normal,
    classification: center.classification,
    condition: center.condition
  };
}

function phi(state, ball, terrainModel) {
  const terrain = evaluateTerrain(terrainModel, state.position.x, state.position.z);
  return state.position.y - terrain.heightM - ball.radiusM;
}

function makeGustState(seedValue) {
  return {
    eta: { x: 0, y: 0, z: 0 },
    rngState: BigInt.asUintN(64, typeof seedValue === "bigint" ? seedValue : BigInt(seedValue)),
    spare: null
  };
}

function cloneGustState(gustState) {
  return {
    eta: cloneVector(gustState.eta),
    rngState: gustState.rngState,
    spare: gustState.spare
  };
}

function nextGustUint32(gustState) {
  gustState.rngState = BigInt.asUintN(64, gustState.rngState + 0x9e3779b97f4a7c15n);
  let z = gustState.rngState;
  z = BigInt.asUintN(64, (z ^ (z >> 30n)) * 0xbf58476d1ce4e5b9n);
  z = BigInt.asUintN(64, (z ^ (z >> 27n)) * 0x94d049bb133111ebn);
  z ^= z >> 31n;
  return Number(z & 0xffffffffn) / 0x100000000;
}

function nextGustNormal(gustState) {
  if (gustState.spare !== null) {
    const cached = gustState.spare;
    gustState.spare = null;
    return cached;
  }
  let u1 = 0;
  while (u1 <= Number.EPSILON) {
    u1 = nextGustUint32(gustState);
  }
  const u2 = nextGustUint32(gustState);
  const radius = Math.sqrt(-2 * Math.log(u1));
  const theta = 2 * Math.PI * u2;
  gustState.spare = radius * Math.sin(theta);
  return radius * Math.cos(theta);
}

function updateGustState(gustState, windModel, dtSeconds) {
  if (dot(windModel.gustSigmaMps, windModel.gustSigmaMps) <= 0 || dtSeconds <= 0) {
    return;
  }
  const tau = Math.max(windModel.gustCorrelationTimeS, 1e-6);
  const a = Math.exp(-dtSeconds / tau);
  const noiseScale = Math.sqrt(Math.max(0, 1 - a * a));

  gustState.eta = {
    x: a * gustState.eta.x + windModel.gustSigmaMps.x * noiseScale * nextGustNormal(gustState),
    y: a * gustState.eta.y + windModel.gustSigmaMps.y * noiseScale * nextGustNormal(gustState),
    z: a * gustState.eta.z + windModel.gustSigmaMps.z * noiseScale * nextGustNormal(gustState)
  };
}

function meanWind(position, windModel, terrain) {
  const hAgl = position.y - terrain.heightM;
  const z0 = Math.max(windModel.roughnessLengthM, 1e-6);
  const yRef = Math.max(windModel.referenceHeightM, z0 + windModel.zeroPlaneDisplacementM + 1e-3);
  const hEff = Math.max(hAgl, z0 + windModel.zeroPlaneDisplacementM + 1e-3);
  const numerator = Math.log((hEff - windModel.zeroPlaneDisplacementM) / z0);
  const denominator = Math.log((yRef - windModel.zeroPlaneDisplacementM) / z0);
  const scale = numerator / denominator;
  return {
    x: scale * windModel.referenceWindMps.x,
    y: windModel.verticalMeanWindMps,
    z: scale * windModel.referenceWindMps.z
  };
}

export function evaluateSurface(terrain) {
  const table = surfaceRows.get(terrain.classification) ?? surfaceRows.get(surfaceClass.fairway);
  const f = clamp(terrain.condition.firmness, 0, 1);
  const w = clamp(terrain.condition.wetness, 0, 1);
  const g = clamp(terrain.condition.grassLengthIndex, 0, 1);
  const eRaw = table.eBase * (1 + 0.45 * (f - 0.5) - 0.35 * w);
  const muKRaw = table.muKBase * (1 + 0.35 * g - 0.10 * w);
  const cRrRaw = table.cRrBase * (1 + 0.90 * g + 0.80 * w - 0.30 * f);
  const eCond = clamp(eRaw, table.eMin, table.eMax);
  const muK = Math.max(muKRaw, 0.05);
  const cRr = Math.max(cRrRaw, 0.001);
  return {
    table,
    eCond,
    muK,
    muS: 1.2 * muK,
    cRr
  };
}

function speedDependentRestitution(surface, classification, vNMinus) {
  const magnitude = Math.abs(vNMinus);
  if (turfLikeSurfaceClasses.has(classification)) {
    return clamp(surface.eCond - 0.010 * Math.max(magnitude - 1.0, 0), surface.table.eMin, surface.table.eMax);
  }
  if (classification === surfaceClass.sand) {
    return clamp(surface.eCond - 0.020 * Math.max(magnitude - 0.5, 0), surface.table.eMin, surface.table.eMax);
  }
  return clamp(surface.eCond - 0.015 * Math.max(magnitude - 1.0, 0), surface.table.eMin, surface.table.eMax);
}

function effectiveImpulseNormal(terrain, vTPhysicalMinus, vNPhysicalMinus) {
  if (!turfLikeSurfaceClasses.has(terrain.classification) || norm(vTPhysicalMinus) < 1e-9) {
    return terrain.normal;
  }
  const table = surfaceRows.get(terrain.classification);
  const alpha =
    (table.tiltMaxDeg * Math.PI) /
      180 *
      clamp(terrain.condition.firmness, 0, 1) *
      (1 - clamp(terrain.condition.wetness, 0, 1)) *
      Math.min(1, Math.abs(vNPhysicalMinus) / 6);
  const tangentHat = tryNormalizeUnitVector(vTPhysicalMinus) ?? { x: 0, y: 0, z: 1 };
  return (
    tryNormalizeUnitVector(addVector(terrain.normal, scaleVector(tangentHat, Math.tan(alpha)))) ??
    terrain.normal
  );
}

function makeState(position, velocity, omega) {
  return { position, velocity, omega };
}

function addScaledState(state, delta, scale) {
  return {
    position: addVector(state.position, scaleVector(delta.dr, scale)),
    velocity: addVector(state.velocity, scaleVector(delta.dv, scale)),
    omega: addVector(state.omega, scaleVector(delta.dOmega, scale))
  };
}

function stateComponents(state) {
  return [
    state.position.x,
    state.position.y,
    state.position.z,
    state.velocity.x,
    state.velocity.y,
    state.velocity.z,
    state.omega.x,
    state.omega.y,
    state.omega.z
  ];
}

function rkErrorNorm(previous, next, errorState) {
  const y0 = stateComponents(previous);
  const y1 = stateComponents(next);
  const err = stateComponents(errorState);
  let maxNorm = 0;
  for (let index = 0; index < err.length; index += 1) {
    const absTol = index < 3 ? positionAbsTolerance : index < 6 ? velocityAbsTolerance : angularVelocityAbsTolerance;
    const denom = absTol + relativeTolerance * Math.max(Math.abs(y0[index]), Math.abs(y1[index]));
    maxNorm = Math.max(maxNorm, Math.abs(err[index]) / denom);
  }
  return maxNorm;
}

function evaluateFlightDerivative(state, environment, constants, eta, overrides) {
  const terrain = evaluateTerrain(environment.terrainModel, state.position.x, state.position.z);
  const windVector = addVector(meanWind(state.position, environment.windModel, terrain), eta);
  const relativeAir = subtractVector(state.velocity, windVector);
  const relativeSpeed = norm(relativeAir);

  let dragForce = { x: 0, y: 0, z: 0 };
  let magnusForce = { x: 0, y: 0, z: 0 };
  let dOmega = { x: 0, y: 0, z: 0 };

  if (!overrides.disableAerodynamics && relativeSpeed >= 1e-6) {
    const uHat = tryNormalizeUnitVector(relativeAir) ?? { x: 0, y: 0, z: 1 };
    const omegaPerp = subtractVector(state.omega, scaleVector(uHat, dot(state.omega, uHat)));
    const reynoldsNumber = constants.densityKgPerM3 * relativeSpeed * environment.ball.diameterM / constants.viscosityPaS;
    const spinRatio = clamp(environment.ball.radiusM * norm(state.omega) / Math.max(relativeSpeed, 1), 0, 1.5);

    const cdSs = 0.280 + 0.070 * spinRatio + 0.135 * Math.sin(Math.PI * (reynoldsNumber - 0.645e5) / 0.378e5);
    const clSs = 0.54 * Math.pow(Math.max(spinRatio, 1e-12), 0.4);
    const sigmaRe = 1 / (1 + Math.exp(-(reynoldsNumber - 0.80e5) / 0.12e5));
    const cdFb = (0.47 - (0.47 - 0.23) * sigmaRe) * (1 + 0.12 * spinRatio * spinRatio);
    const clFb = 0.33 * spinRatio / (spinRatio + 0.10);
    const blend = clamp((relativeSpeed - 12.0) / 8.0, 0, 1);
    const cd = clamp(blend * cdSs + (1 - blend) * cdFb, 0.15, 0.65);
    const cl = clamp(blend * clSs + (1 - blend) * clFb, 0, 0.40);

    dragForce = scaleVector(uHat, -0.5 * constants.densityKgPerM3 * environment.ball.areaM2 * cd * relativeSpeed * relativeSpeed);
    if (norm(omegaPerp) >= 1e-9) {
      const mHat = tryNormalizeUnitVector(cross(omegaPerp, uHat)) ?? { x: 0, y: 1, z: 0 };
      magnusForce = scaleVector(mHat, 0.5 * constants.densityKgPerM3 * environment.ball.areaM2 * cl * relativeSpeed * relativeSpeed);
    }
  }

  if (!overrides.disableSpinDecay && relativeSpeed >= 1e-6) {
    dOmega = scaleVector(state.omega, -(2e-5 / environment.ball.radiusM) * relativeSpeed);
  }

  return {
    dr: state.velocity,
    dv: addVector(gravityVector, scaleVector(addVector(dragForce, magnusForce), 1 / environment.ball.massKg)),
    dOmega
  };
}

function rk45Step(state, dtSeconds, environment, constants, eta, overrides) {
  const k1 = evaluateFlightDerivative(state, environment, constants, eta, overrides);
  const k2 = evaluateFlightDerivative(addScaledState(state, k1, dtSeconds * (1 / 5)), environment, constants, eta, overrides);
  const k3 = evaluateFlightDerivative(
    makeState(
      addVector(state.position, scaleVector(addVector(scaleVector(k1.dr, 3 / 40), scaleVector(k2.dr, 9 / 40)), dtSeconds)),
      addVector(state.velocity, scaleVector(addVector(scaleVector(k1.dv, 3 / 40), scaleVector(k2.dv, 9 / 40)), dtSeconds)),
      addVector(state.omega, scaleVector(addVector(scaleVector(k1.dOmega, 3 / 40), scaleVector(k2.dOmega, 9 / 40)), dtSeconds))
    ),
    environment,
    constants,
    eta,
    overrides
  );
  const k4 = evaluateFlightDerivative(
    makeState(
      addVector(
        state.position,
        scaleVector(
          addVector(
            addVector(scaleVector(k1.dr, 44 / 45), scaleVector(k2.dr, -56 / 15)),
            scaleVector(k3.dr, 32 / 9)
          ),
          dtSeconds
        )
      ),
      addVector(
        state.velocity,
        scaleVector(
          addVector(
            addVector(scaleVector(k1.dv, 44 / 45), scaleVector(k2.dv, -56 / 15)),
            scaleVector(k3.dv, 32 / 9)
          ),
          dtSeconds
        )
      ),
      addVector(
        state.omega,
        scaleVector(
          addVector(
            addVector(scaleVector(k1.dOmega, 44 / 45), scaleVector(k2.dOmega, -56 / 15)),
            scaleVector(k3.dOmega, 32 / 9)
          ),
          dtSeconds
        )
      )
    ),
    environment,
    constants,
    eta,
    overrides
  );
  const k5 = evaluateFlightDerivative(
    makeState(
      addVector(
        state.position,
        scaleVector(
          addVector(
            addVector(
              addVector(scaleVector(k1.dr, 19372 / 6561), scaleVector(k2.dr, -25360 / 2187)),
              scaleVector(k3.dr, 64448 / 6561)
            ),
            scaleVector(k4.dr, -212 / 729)
          ),
          dtSeconds
        )
      ),
      addVector(
        state.velocity,
        scaleVector(
          addVector(
            addVector(
              addVector(scaleVector(k1.dv, 19372 / 6561), scaleVector(k2.dv, -25360 / 2187)),
              scaleVector(k3.dv, 64448 / 6561)
            ),
            scaleVector(k4.dv, -212 / 729)
          ),
          dtSeconds
        )
      ),
      addVector(
        state.omega,
        scaleVector(
          addVector(
            addVector(
              addVector(scaleVector(k1.dOmega, 19372 / 6561), scaleVector(k2.dOmega, -25360 / 2187)),
              scaleVector(k3.dOmega, 64448 / 6561)
            ),
            scaleVector(k4.dOmega, -212 / 729)
          ),
          dtSeconds
        )
      )
    ),
    environment,
    constants,
    eta,
    overrides
  );
  const k6 = evaluateFlightDerivative(
    makeState(
      addVector(
        state.position,
        scaleVector(
          addVector(
            addVector(
              addVector(
                addVector(scaleVector(k1.dr, 9017 / 3168), scaleVector(k2.dr, -355 / 33)),
                scaleVector(k3.dr, 46732 / 5247)
              ),
              scaleVector(k4.dr, 49 / 176)
            ),
            scaleVector(k5.dr, -5103 / 18656)
          ),
          dtSeconds
        )
      ),
      addVector(
        state.velocity,
        scaleVector(
          addVector(
            addVector(
              addVector(
                addVector(scaleVector(k1.dv, 9017 / 3168), scaleVector(k2.dv, -355 / 33)),
                scaleVector(k3.dv, 46732 / 5247)
              ),
              scaleVector(k4.dv, 49 / 176)
            ),
            scaleVector(k5.dv, -5103 / 18656)
          ),
          dtSeconds
        )
      ),
      addVector(
        state.omega,
        scaleVector(
          addVector(
            addVector(
              addVector(
                addVector(scaleVector(k1.dOmega, 9017 / 3168), scaleVector(k2.dOmega, -355 / 33)),
                scaleVector(k3.dOmega, 46732 / 5247)
              ),
              scaleVector(k4.dOmega, 49 / 176)
            ),
            scaleVector(k5.dOmega, -5103 / 18656)
          ),
          dtSeconds
        )
      )
    ),
    environment,
    constants,
    eta,
    overrides
  );
  const k7State = makeState(
    addVector(
      state.position,
      scaleVector(
        addVector(
          addVector(
            addVector(scaleVector(k1.dr, 35 / 384), scaleVector(k3.dr, 500 / 1113)),
            scaleVector(k4.dr, 125 / 192)
          ),
          addVector(scaleVector(k5.dr, -2187 / 6784), scaleVector(k6.dr, 11 / 84))
        ),
        dtSeconds
      )
    ),
    addVector(
      state.velocity,
      scaleVector(
        addVector(
          addVector(
            addVector(scaleVector(k1.dv, 35 / 384), scaleVector(k3.dv, 500 / 1113)),
            scaleVector(k4.dv, 125 / 192)
          ),
          addVector(scaleVector(k5.dv, -2187 / 6784), scaleVector(k6.dv, 11 / 84))
        ),
        dtSeconds
      )
    ),
    addVector(
      state.omega,
      scaleVector(
        addVector(
          addVector(
            addVector(scaleVector(k1.dOmega, 35 / 384), scaleVector(k3.dOmega, 500 / 1113)),
            scaleVector(k4.dOmega, 125 / 192)
          ),
          addVector(scaleVector(k5.dOmega, -2187 / 6784), scaleVector(k6.dOmega, 11 / 84))
        ),
        dtSeconds
      )
    )
  );
  const k7 = evaluateFlightDerivative(k7State, environment, constants, eta, overrides);

  const fifth = k7State;
  const fourth = makeState(
    addVector(
      state.position,
      scaleVector(
        addVector(
          addVector(
            addVector(scaleVector(k1.dr, 5179 / 57600), scaleVector(k3.dr, 7571 / 16695)),
            scaleVector(k4.dr, 393 / 640)
          ),
          addVector(
            addVector(scaleVector(k5.dr, -92097 / 339200), scaleVector(k6.dr, 187 / 2100)),
            scaleVector(k7.dr, 1 / 40)
          )
        ),
        dtSeconds
      )
    ),
    addVector(
      state.velocity,
      scaleVector(
        addVector(
          addVector(
            addVector(scaleVector(k1.dv, 5179 / 57600), scaleVector(k3.dv, 7571 / 16695)),
            scaleVector(k4.dv, 393 / 640)
          ),
          addVector(
            addVector(scaleVector(k5.dv, -92097 / 339200), scaleVector(k6.dv, 187 / 2100)),
            scaleVector(k7.dv, 1 / 40)
          )
        ),
        dtSeconds
      )
    ),
    addVector(
      state.omega,
      scaleVector(
        addVector(
          addVector(
            addVector(scaleVector(k1.dOmega, 5179 / 57600), scaleVector(k3.dOmega, 7571 / 16695)),
            scaleVector(k4.dOmega, 393 / 640)
          ),
          addVector(
            addVector(scaleVector(k5.dOmega, -92097 / 339200), scaleVector(k6.dOmega, 187 / 2100)),
            scaleVector(k7.dOmega, 1 / 40)
          )
        ),
        dtSeconds
      )
    )
  );

  const errorState = makeState(
    subtractVector(fifth.position, fourth.position),
    subtractVector(fifth.velocity, fourth.velocity),
    subtractVector(fifth.omega, fourth.omega)
  );

  return {
    fifth,
    fourth,
    errorNorm: rkErrorNorm(state, fifth, errorState)
  };
}

function suggestNextAirStep(currentStep, errorNorm) {
  if (errorNorm <= 1e-12) {
    return clamp(currentStep * 2, airStepMinimum, airStepMaximum);
  }
  const factor = clamp(0.9 * Math.pow(1 / errorNorm, 0.2), 0.2, 2.0);
  return clamp(currentStep * factor, airStepMinimum, airStepMaximum);
}

function integrateFixedInterval(start, dtSeconds, environment, constants, eta, overrides) {
  return rk45Step(start, dtSeconds, environment, constants, eta, overrides).fifth;
}

function projectToSurface(state, ball, terrainModel) {
  const terrain = evaluateTerrain(terrainModel, state.position.x, state.position.z);
  const tangentialVelocity = subtractVector(state.velocity, scaleVector(terrain.normal, dot(state.velocity, terrain.normal)));
  return makeState(
    { x: state.position.x, y: terrain.heightM + ball.radiusM, z: state.position.z },
    tangentialVelocity,
    cloneVector(state.omega)
  );
}

function makeTraceSample(timeSeconds, state, regime) {
  return {
    timeSeconds,
    regime,
    position: clonePoint(state.position),
    translational: cartesianToTranslationalVelocity(state.velocity),
    rotational: rightHandRuleToRotationalVelocity(state.omega)
  };
}

function contactWithNormal(state, ball, normal) {
  const contactVelocity = subtractVector(state.velocity, scaleVector(cross(state.omega, normal), ball.radiusM));
  const normalSpeed = dot(contactVelocity, normal);
  return {
    contactVelocity,
    normalSpeed,
    tangentialVelocity: subtractVector(contactVelocity, scaleVector(normal, normalSpeed))
  };
}

function solveContactTime(start, end, dtSeconds, environment, constants, eta, overrides) {
  const phiStart = phi(start, environment.ball, environment.terrainModel);
  const phiEnd = phi(end, environment.ball, environment.terrainModel);
  if (!(phiStart > 0 && phiEnd <= 0)) {
    throw new SimulationError("Contact root was not bracketed.", "root_finding_failure");
  }

  let lower = 0;
  let upper = dtSeconds;
  let best = end;

  for (let iteration = 0; iteration < maxRootIterations; iteration += 1) {
    const midpoint = 0.5 * (lower + upper);
    const candidate = integrateFixedInterval(start, midpoint, environment, constants, eta, overrides);
    const phiCandidate = phi(candidate, environment.ball, environment.terrainModel);
    best = candidate;
    if ((upper - lower) <= rootTimeTolerance && Math.abs(phiCandidate) <= rootPhiTolerance) {
      return { state: candidate, dtSeconds: midpoint };
    }
    if (phiCandidate > 0) {
      lower = midpoint;
    } else {
      upper = midpoint;
    }
  }

  const midpoint = 0.5 * (lower + upper);
  const candidate = integrateFixedInterval(start, midpoint, environment, constants, eta, overrides);
  const phiCandidate = phi(candidate, environment.ball, environment.terrainModel);
  if ((upper - lower) > rootTimeTolerance || Math.abs(phiCandidate) > rootPhiTolerance) {
    throw new SimulationError("Failed to converge contact root.", "root_finding_failure");
  }
  return { state: candidate, dtSeconds: midpoint, best };
}

function solveReferencePlaneTime(start, end, dtSeconds, environment, constants, eta, overrides) {
  if (!(start.position.y > 0 && end.position.y <= 0)) {
    throw new SimulationError("Reference-plane root was not bracketed.", "root_finding_failure");
  }

  let lower = 0;
  let upper = dtSeconds;

  for (let iteration = 0; iteration < maxRootIterations; iteration += 1) {
    const midpoint = 0.5 * (lower + upper);
    const candidate = integrateFixedInterval(start, midpoint, environment, constants, eta, overrides);
    if ((upper - lower) <= rootTimeTolerance && Math.abs(candidate.position.y) <= rootPhiTolerance) {
      return { state: makeState({ ...candidate.position, y: 0 }, candidate.velocity, candidate.omega), dtSeconds: midpoint };
    }
    if (candidate.position.y > 0) {
      lower = midpoint;
    } else {
      upper = midpoint;
    }
  }

  const midpoint = 0.5 * (lower + upper);
  const candidate = integrateFixedInterval(start, midpoint, environment, constants, eta, overrides);
  if ((upper - lower) > rootTimeTolerance || Math.abs(candidate.position.y) > rootPhiTolerance) {
    throw new SimulationError("Failed to converge reference-plane root.", "root_finding_failure");
  }
  return { state: makeState({ ...candidate.position, y: 0 }, candidate.velocity, candidate.omega), dtSeconds: midpoint };
}

function solvePeakHeightTime(start, end, dtSeconds, environment, constants, eta, overrides) {
  if (!(start.velocity.y > 0 && end.velocity.y <= 0)) {
    throw new SimulationError("Peak-height root was not bracketed.", "root_finding_failure");
  }

  let lower = 0;
  let upper = dtSeconds;

  for (let iteration = 0; iteration < maxRootIterations; iteration += 1) {
    const midpoint = 0.5 * (lower + upper);
    const candidate = integrateFixedInterval(start, midpoint, environment, constants, eta, overrides);
    if ((upper - lower) <= rootTimeTolerance && Math.abs(candidate.velocity.y) <= rootPhiTolerance) {
      return { state: makeState(candidate.position, { ...candidate.velocity, y: 0 }, candidate.omega), dtSeconds: midpoint };
    }
    if (candidate.velocity.y > 0) {
      lower = midpoint;
    } else {
      upper = midpoint;
    }
  }

  const midpoint = 0.5 * (lower + upper);
  const candidate = integrateFixedInterval(start, midpoint, environment, constants, eta, overrides);
  if ((upper - lower) > rootTimeTolerance || Math.abs(candidate.velocity.y) > rootPhiTolerance) {
    throw new SimulationError("Failed to converge peak-height root.", "root_finding_failure");
  }
  return { state: makeState(candidate.position, { ...candidate.velocity, y: 0 }, candidate.omega), dtSeconds: midpoint };
}

function classifyGroundMode(state, ball, terrain, surface) {
  const physical = contactWithNormal(state, ball, terrain.normal);
  const normalReaction = Math.max(-ball.massKg * dot(gravityVector, terrain.normal), 0);
  const vT = subtractVector(state.velocity, scaleVector(terrain.normal, dot(state.velocity, terrain.normal)));
  const gT = subtractVector(gravityVector, scaleVector(terrain.normal, dot(gravityVector, terrain.normal)));

  if (terrain.classification === surfaceClass.sand && norm(state.velocity) < 0.10) {
    return modes.rest;
  }
  if (norm(state.velocity) < 0.02 && norm(state.omega) < 1.0 && norm(gT) <= surface.cRr * normalReaction / ball.massKg + 0.01) {
    return modes.rest;
  }

  const sMag = norm(physical.tangentialVelocity);
  if (norm(vT) < 1e-9 && sMag <= nearPureRollSlipTolerance) {
    return modes.pureRoll;
  }
  if (sMag > 0.05) {
    return modes.skid;
  }
  if (sMag > 0.005) {
    return modes.rollWithSlip;
  }
  return modes.pureRoll;
}

function tangentialComponent(value, normal) {
  return subtractVector(value, scaleVector(normal, dot(value, normal)));
}

function suppressTangentialReversal(previousTangential, candidate, normal, slipMagnitude = Number.POSITIVE_INFINITY) {
  const nextTangential = tangentialComponent(candidate, normal);
  if (norm(previousTangential) <= 1e-9 && slipMagnitude <= nearPureRollSlipTolerance) {
    return subtractVector(candidate, nextTangential);
  }
  if (norm(previousTangential) > 1e-9 && dot(previousTangential, nextTangential) < 0) {
    return subtractVector(candidate, nextTangential);
  }
  return candidate;
}

export function advanceGroundStep(state, ball, terrain, surface, currentMode) {
  const physical = contactWithNormal(state, ball, terrain.normal);
  const normalReaction = Math.max(-ball.massKg * dot(gravityVector, terrain.normal), 0);
  const vT = tangentialComponent(state.velocity, terrain.normal);

  if (currentMode === modes.pureRoll) {
    const vHatT = norm(vT) < 1e-9 ? { x: 0, y: 0, z: 0 } : (tryNormalizeUnitVector(vT) ?? { x: 0, y: 0, z: 0 });
    const gT = subtractVector(gravityVector, scaleVector(terrain.normal, dot(gravityVector, terrain.normal)));
    const aRoll = scaleVector(
      subtractVector(gT, scaleVector(vHatT, (surface.cRr * normalReaction) / ball.massKg)),
      1 / (1 + ball.inertiaKgM2 / (ball.massKg * ball.radiusM * ball.radiusM))
    );
    const vNext = suppressTangentialReversal(vT, addVector(vT, scaleVector(aRoll, groundStepSeconds)), terrain.normal);
    return makeState(
      addVector(state.position, scaleVector(vNext, groundStepSeconds)),
      vNext,
      scaleVector(cross(terrain.normal, vNext), 1 / ball.radiusM)
    );
  }

  let frictionForce = { x: 0, y: 0, z: 0 };
  if (norm(physical.tangentialVelocity) >= 1e-9) {
    const sHat = tryNormalizeUnitVector(physical.tangentialVelocity) ?? { x: 0, y: 0, z: 1 };
    frictionForce = scaleVector(sHat, -surface.muK * normalReaction);
  }

  let rollingResistance = { x: 0, y: 0, z: 0 };
  if (norm(vT) >= 1e-9) {
    const vHat = tryNormalizeUnitVector(vT) ?? { x: 0, y: 0, z: 1 };
    rollingResistance = scaleVector(vHat, -surface.cRr * normalReaction);
  }

  const totalForce = addVector(
    addVector(scaleVector(gravityVector, ball.massKg), scaleVector(terrain.normal, normalReaction)),
    addVector(frictionForce, rollingResistance)
  );
  const torque = scaleVector(cross(terrain.normal, frictionForce), -ball.radiusM);
  const vNext = suppressTangentialReversal(
    vT,
    addVector(state.velocity, scaleVector(totalForce, groundStepSeconds / ball.massKg)),
    terrain.normal,
    norm(physical.tangentialVelocity)
  );
  const omegaNext = addVector(state.omega, scaleVector(torque, groundStepSeconds / ball.inertiaKgM2));

  return makeState(
    addVector(state.position, scaleVector(vNext, groundStepSeconds)),
    vNext,
    omegaNext
  );
}

function normalizeEnvironmentAndLaunch(launchStateInput, worldInput) {
  const environment = normalizeWorld(worldInput);
  const launchState = normalizeLaunchState(launchStateInput);
  const constants = evaluateEnvironmentConstants(environment.air);
  return { environment, launchState, constants };
}

function makeLaunchContext(environment, launchState, constants) {
  const state = makeState(
    { x: 0, y: 0, z: 0 },
    translationalVelocityToCartesian(launchState.translational),
    rotationalVelocityToRightHandRule(launchState.rotational)
  );
  return {
    environment,
    constants,
    state,
    timeSeconds: 0,
    airStep: 1e-3,
    currentMode: modes.flight,
    gustState: makeGustState(environment.windModel.gustSeed),
    trace: [makeTraceSample(0, state, traceRegime.flight)],
    firstImpact: null,
    bounceTerminationCounter: 0
  };
}

function handleImmediateSurfaceContact(context, environment) {
  const terrain = evaluateTerrain(environment.terrainModel, context.state.position.x, context.state.position.z);
  const clearance = context.state.position.y - terrain.heightM - environment.ball.radiusM;
  if (clearance > rootPhiTolerance) {
    return;
  }

  context.state = makeState(
    { x: context.state.position.x, y: terrain.heightM + environment.ball.radiusM, z: context.state.position.z },
    cloneVector(context.state.velocity),
    cloneVector(context.state.omega)
  );
  context.trace[0] = makeTraceSample(context.timeSeconds, context.state, traceRegime.flight);
  handleTerrainContact(context, environment);
}

function handleTerrainContact(context, environment) {
  const terrain = evaluateTerrain(environment.terrainModel, context.state.position.x, context.state.position.z);
  const physicalContact = contactWithNormal(context.state, environment.ball, terrain.normal);

  if (context.firstImpact === null) {
    context.firstImpact = {
      point: subtractVector(context.state.position, scaleVector(terrain.normal, environment.ball.radiusM)),
      timeSeconds: context.timeSeconds,
      incomingDirection: tryNormalizeUnitVector(context.state.velocity) ?? { x: 0, y: 0, z: 1 }
    };
  }

  if (Math.abs(physicalContact.normalSpeed) < 0.2) {
    context.state = projectToSurface(context.state, environment.ball, environment.terrainModel);
    context.currentMode = modes.skid;
    return;
  }

  const surface = evaluateSurface(terrain);
  const impulseNormal = effectiveImpulseNormal(terrain, physicalContact.tangentialVelocity, physicalContact.normalSpeed);
  const impulseContact = contactWithNormal(context.state, environment.ball, impulseNormal);
  if (impulseContact.normalSpeed >= 0) {
    context.state = projectToSurface(context.state, environment.ball, environment.terrainModel);
    context.currentMode = modes.skid;
    return;
  }

  const restitution = speedDependentRestitution(surface, terrain.classification, impulseContact.normalSpeed);
  const jN = -environment.ball.massKg * (1 + restitution) * impulseContact.normalSpeed;
  const jTStick = scaleVector(
    impulseContact.tangentialVelocity,
    -1 / (1 / environment.ball.massKg + (environment.ball.radiusM * environment.ball.radiusM) / environment.ball.inertiaKgM2)
  );

  let jT = { x: 0, y: 0, z: 0 };
  if (norm(jTStick) <= surface.muS * jN) {
    jT = jTStick;
  } else if (norm(impulseContact.tangentialVelocity) >= 1e-9) {
    const vHatT = tryNormalizeUnitVector(impulseContact.tangentialVelocity) ?? { x: 0, y: 0, z: 1 };
    jT = scaleVector(vHatT, -surface.muK * jN);
  }

  context.state = makeState(
    clonePoint(context.state.position),
    addVector(context.state.velocity, scaleVector(addVector(scaleVector(impulseNormal, jN), jT), 1 / environment.ball.massKg)),
    addVector(context.state.omega, scaleVector(cross(impulseNormal, jT), -environment.ball.radiusM / environment.ball.inertiaKgM2))
  );
  context.trace.push(makeTraceSample(context.timeSeconds, context.state, traceRegime.flight));

  const postContact = contactWithNormal(context.state, environment.ball, terrain.normal);
  const gN = Math.max(-dot(gravityVector, terrain.normal), 1e-9);
  const vNPlus = Math.max(postContact.normalSpeed, 0);
  const tHop = (2 * vNPlus) / gN;
  const hHop = (vNPlus * vNPlus) / (2 * gN);
  const terminationTrigger = vNPlus < 0.3 || tHop < 0.03 || hHop < 0.002;
  context.bounceTerminationCounter = terminationTrigger ? context.bounceTerminationCounter + 1 : 0;

  const forceGround =
    context.bounceTerminationCounter >= 2 ||
    !(hHop > 0.005) ||
    (terrain.classification === surfaceClass.sand && (hHop < 0.01 || norm(context.state.velocity) < 1));

  if (forceGround) {
    context.state = projectToSurface(context.state, environment.ball, environment.terrainModel);
    context.currentMode = modes.skid;
    return;
  }

  context.state = makeState(
    addVector(context.state.position, scaleVector(terrain.normal, 1e-7)),
    cloneVector(context.state.velocity),
    cloneVector(context.state.omega)
  );
  context.currentMode = modes.hop;
  context.airStep = clamp(Math.min(context.airStep, tHop), airStepMinimum, airStepMaximum);
}

function continueSimulation(context, overrides) {
  const { environment, constants } = context;

  for (let airIteration = 0; context.currentMode === modes.flight || context.currentMode === modes.hop; airIteration += 1) {
    if (airIteration >= maxAirSteps) {
      throw new SimulationError("Exceeded airborne step budget.", "integration_failure");
    }

    const step = rk45Step(context.state, context.airStep, environment, constants, context.gustState.eta, overrides);
    if (!Number.isFinite(step.errorNorm)) {
      throw new SimulationError("Encountered non-finite RK error estimate.", "integration_failure");
    }
    if (step.errorNorm > 1) {
      context.airStep = suggestNextAirStep(context.airStep, step.errorNorm);
      if (context.airStep <= airStepMinimum + 1e-12) {
        throw new SimulationError("Adaptive RK step underflow.", "integration_failure");
      }
      continue;
    }

    const phiNext = phi(step.fifth, environment.ball, environment.terrainModel);
    if (phiNext > 0) {
      context.timeSeconds += context.airStep;
      context.state = step.fifth;
      context.trace.push(makeTraceSample(context.timeSeconds, context.state, traceRegime.flight));
      updateGustState(context.gustState, environment.windModel, context.airStep);
      context.airStep = suggestNextAirStep(context.airStep, Math.max(step.errorNorm, 1e-12));
      continue;
    }

    const phiCurrent = phi(context.state, environment.ball, environment.terrainModel);
    if (phiCurrent <= rootPhiTolerance) {
      handleImmediateSurfaceContact(context, environment);
      continue;
    }

    const root = solveContactTime(context.state, step.fifth, context.airStep, environment, constants, context.gustState.eta, overrides);
    context.timeSeconds += root.dtSeconds;
    context.state = root.state;
    context.trace.push(makeTraceSample(context.timeSeconds, context.state, traceRegime.flight));
    updateGustState(context.gustState, environment.windModel, root.dtSeconds);
    handleTerrainContact(context, environment);
  }

  if (context.firstImpact === null) {
    throw new SimulationError("Simulation completed without a ground impact.", "simulation_failure");
  }

  for (let groundIteration = 0; context.currentMode !== modes.rest; groundIteration += 1) {
    if (groundIteration >= maxGroundSteps) {
      throw new SimulationError("Exceeded sustained-contact step budget.", "integration_failure");
    }

    const terrain = evaluateTerrain(environment.terrainModel, context.state.position.x, context.state.position.z);
    const surface = evaluateSurface(terrain);
    context.currentMode = classifyGroundMode(context.state, environment.ball, terrain, surface);
    if (context.currentMode === modes.rest) {
      break;
    }
    context.state = advanceGroundStep(context.state, environment.ball, terrain, surface, context.currentMode);
    context.state = projectToSurface(context.state, environment.ball, environment.terrainModel);
    context.timeSeconds += groundStepSeconds;
    context.trace.push(makeTraceSample(context.timeSeconds, context.state, traceRegime.roll));
  }

  context.trace[context.trace.length - 1].regime = traceRegime.stop;
  const finalTerrain = evaluateTerrain(environment.terrainModel, context.state.position.x, context.state.position.z);
  return {
    impact: context.firstImpact,
    finalRestPoint: subtractVector(context.state.position, scaleVector(finalTerrain.normal, environment.ball.radiusM)),
    trace: context.trace
  };
}

function vectorEquals(left, right) {
  return left.x === right.x && left.y === right.y && left.z === right.z;
}

function nonTerrainEnvironmentEquals(left, right) {
  return left.ball.massKg === right.ball.massKg &&
    left.ball.diameterM === right.ball.diameterM &&
    left.ball.radiusM === right.ball.radiusM &&
    left.ball.areaM2 === right.ball.areaM2 &&
    left.ball.inertiaKgM2 === right.ball.inertiaKgM2 &&
    left.ball.aerodynamicProfile === right.ball.aerodynamicProfile &&
    left.air.temperatureKelvin === right.air.temperatureKelvin &&
    left.air.pressurePascals === right.air.pressurePascals &&
    left.air.altitudeMetres === right.air.altitudeMetres &&
    left.air.relativeHumidity === right.air.relativeHumidity &&
    vectorEquals(left.windModel.referenceWindMps, right.windModel.referenceWindMps) &&
    left.windModel.verticalMeanWindMps === right.windModel.verticalMeanWindMps &&
    left.windModel.referenceHeightM === right.windModel.referenceHeightM &&
    left.windModel.roughnessLengthM === right.windModel.roughnessLengthM &&
    left.windModel.zeroPlaneDisplacementM === right.windModel.zeroPlaneDisplacementM &&
    left.windModel.gustCorrelationTimeS === right.windModel.gustCorrelationTimeS &&
    vectorEquals(left.windModel.gustSigmaMps, right.windModel.gustSigmaMps) &&
    left.windModel.gustSeed === right.windModel.gustSeed;
}

function cloneEnvironment(environment) {
  return {
    ball: { ...environment.ball },
    air: { ...environment.air },
    windModel: {
      ...environment.windModel,
      referenceWindMps: cloneVector(environment.windModel.referenceWindMps),
      gustSigmaMps: cloneVector(environment.windModel.gustSigmaMps)
    },
    terrainModel: environment.terrainModel
  };
}

function makeFlightCheckpoint(payload, kind) {
  const checkpoint = /** @type {FlightCheckpoint} */ (Object.freeze(new FlightCheckpointToken()));
  flightCheckpointPayloads.set(checkpoint, {
    kind,
    environment: cloneEnvironment(payload.environment),
    state: cloneState(payload.state),
    timeSeconds: payload.timeSeconds,
    airStep: payload.airStep,
    gustState: cloneGustState(payload.gustState)
  });
  return checkpoint;
}

function getFlightCheckpointPayload(checkpoint) {
  const payload = flightCheckpointPayloads.get(checkpoint);
  if (payload === undefined) {
    throw new TypeError("Expected an opaque ni_slow flight checkpoint.");
  }
  return payload;
}

/**
 * @param {LaunchStateInput} launchStateInput
 * @param {WorldInput} worldInput
 * @param {SimulationOverrides} overrides
 * @returns {SimulationResult}
 */
export function simulateInternal(launchStateInput, worldInput, overrides = {}) {
  const { environment, launchState, constants } = normalizeEnvironmentAndLaunch(launchStateInput, worldInput);
  return continueSimulation(makeLaunchContext(environment, launchState, constants), overrides);
}

/**
 * @param {LaunchStateInput} launchStateInput
 * @param {WorldInput} worldInput
 * @param {SimulationOverrides} overrides
 * @returns {FlightCheckpoint}
 */
export function createReferencePlaneCheckpointInternal(launchStateInput, worldInput, overrides = {}) {
  const { environment, launchState, constants } = normalizeEnvironmentAndLaunch(launchStateInput, worldInput);
  const context = makeLaunchContext(environment, launchState, constants);

  for (let airIteration = 0; airIteration < maxAirSteps; airIteration += 1) {
    const step = rk45Step(context.state, context.airStep, environment, constants, context.gustState.eta, overrides);
    if (!Number.isFinite(step.errorNorm)) {
      throw new SimulationError("Encountered non-finite RK error estimate.", "integration_failure");
    }
    if (step.errorNorm > 1) {
      context.airStep = suggestNextAirStep(context.airStep, step.errorNorm);
      if (context.airStep <= airStepMinimum + 1e-12) {
        throw new SimulationError("Adaptive RK step underflow.", "integration_failure");
      }
      continue;
    }

    if (context.state.position.y > 0 && step.fifth.position.y <= 0) {
      const root = solveReferencePlaneTime(context.state, step.fifth, context.airStep, environment, constants, context.gustState.eta, overrides);
      context.timeSeconds += root.dtSeconds;
      context.state = root.state;
      updateGustState(context.gustState, environment.windModel, root.dtSeconds);

      const clearance = phi(context.state, environment.ball, environment.terrainModel);
      if (clearance < -rootPhiTolerance) {
        throw new SimulationError("Reference-plane checkpoint lies inside the source terrain.", "simulation_failure");
      }

      return makeFlightCheckpoint(context, "reference_plane");
    }

    const phiNext = phi(step.fifth, environment.ball, environment.terrainModel);
    if (phiNext <= 0) {
      throw new SimulationError("Terrain contact occurred before the reference-plane crossing.", "simulation_failure");
    }

    context.timeSeconds += context.airStep;
    context.state = step.fifth;
    updateGustState(context.gustState, environment.windModel, context.airStep);
    context.airStep = suggestNextAirStep(context.airStep, Math.max(step.errorNorm, 1e-12));
  }

  throw new SimulationError("Exceeded airborne step budget before the reference-plane crossing.", "integration_failure");
}

/**
 * @param {LaunchStateInput} launchStateInput
 * @param {WorldInput} worldInput
 * @param {SimulationOverrides} overrides
 * @returns {FlightCheckpoint}
 */
export function createPeakHeightCheckpointInternal(launchStateInput, worldInput, overrides = {}) {
  const { environment, launchState, constants } = normalizeEnvironmentAndLaunch(launchStateInput, worldInput);
  const context = makeLaunchContext(environment, launchState, constants);

  if (context.state.velocity.y <= 0) {
    return makeFlightCheckpoint(context, "peak_height");
  }

  for (let airIteration = 0; airIteration < maxAirSteps; airIteration += 1) {
    const step = rk45Step(context.state, context.airStep, environment, constants, context.gustState.eta, overrides);
    if (!Number.isFinite(step.errorNorm)) {
      throw new SimulationError("Encountered non-finite RK error estimate.", "integration_failure");
    }
    if (step.errorNorm > 1) {
      context.airStep = suggestNextAirStep(context.airStep, step.errorNorm);
      if (context.airStep <= airStepMinimum + 1e-12) {
        throw new SimulationError("Adaptive RK step underflow.", "integration_failure");
      }
      continue;
    }

    const phiNext = phi(step.fifth, environment.ball, environment.terrainModel);
    if (context.state.velocity.y > 0 && step.fifth.velocity.y <= 0) {
      const root = solvePeakHeightTime(context.state, step.fifth, context.airStep, environment, constants, context.gustState.eta, overrides);
      if (phiNext <= 0) {
        const contactRoot = solveContactTime(context.state, step.fifth, context.airStep, environment, constants, context.gustState.eta, overrides);
        if (contactRoot.dtSeconds + rootTimeTolerance < root.dtSeconds) {
          throw new SimulationError("Terrain contact occurred before the peak-height checkpoint.", "simulation_failure");
        }
      }
      context.timeSeconds += root.dtSeconds;
      context.state = root.state;
      updateGustState(context.gustState, environment.windModel, root.dtSeconds);

      const clearance = phi(context.state, environment.ball, environment.terrainModel);
      if (clearance < -rootPhiTolerance) {
        throw new SimulationError("Peak-height checkpoint lies inside the source terrain.", "simulation_failure");
      }

      return makeFlightCheckpoint(context, "peak_height");
    }

    if (phiNext <= 0) {
      throw new SimulationError("Terrain contact occurred before the peak-height checkpoint.", "simulation_failure");
    }

    context.timeSeconds += context.airStep;
    context.state = step.fifth;
    updateGustState(context.gustState, environment.windModel, context.airStep);
    context.airStep = suggestNextAirStep(context.airStep, Math.max(step.errorNorm, 1e-12));
  }

  throw new SimulationError("Exceeded airborne step budget before the peak-height checkpoint.", "integration_failure");
}

/**
 * @param {FlightCheckpoint} checkpoint
 * @param {WorldInput} worldInput
 * @param {SimulationOverrides} overrides
 * @returns {SimulationResult}
 */
export function resumeFromFlightCheckpointInternal(checkpoint, worldInput, overrides = {}) {
  const payload = getFlightCheckpointPayload(checkpoint);
  const environment = normalizeWorld(worldInput, { enforceLaunchClearance: false });
  if (!nonTerrainEnvironmentEquals(payload.environment, environment)) {
    throw new TypeError("Checkpoint resume world must match the checkpoint's ball, atmosphere, and wind configuration.");
  }
  const constants = evaluateEnvironmentConstants(environment.air);
  const state = cloneState(payload.state);
  const terrain = evaluateTerrain(environment.terrainModel, state.position.x, state.position.z);
  const clearance = state.position.y - terrain.heightM - environment.ball.radiusM;
  if (clearance < -rootPhiTolerance) {
    throw new TypeError("Checkpoint resume terrain is above the allowed surface at the checkpoint.");
  }

  const context = {
    environment,
    constants,
    state,
    timeSeconds: payload.timeSeconds,
    airStep: payload.airStep,
    currentMode: modes.flight,
    gustState: cloneGustState(payload.gustState),
    trace: [makeTraceSample(payload.timeSeconds, state, traceRegime.flight)],
    firstImpact: null,
    bounceTerminationCounter: 0
  };

  handleImmediateSurfaceContact(context, environment);

  return continueSimulation(context, overrides);
}

/**
 * @param {WorldInput} worldInput
 * @returns {Simulator}
 */
export function createSimulatorInternal(worldInput) {
  const environment = normalizeWorld(worldInput);
  return Object.freeze({
    environment,
    simulate(launchState) {
      return simulateInternal(launchState, environment, {});
    },
    createReferencePlaneCheckpoint(launchState) {
      return createReferencePlaneCheckpointInternal(launchState, environment, {});
    },
    createPeakHeightCheckpoint(launchState) {
      return createPeakHeightCheckpointInternal(launchState, environment, {});
    },
    resumeFromFlightCheckpoint(checkpoint) {
      return resumeFromFlightCheckpointInternal(checkpoint, environment, {});
    }
  });
}

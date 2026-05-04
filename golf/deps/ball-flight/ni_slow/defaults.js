import { surfaceClass } from "../core/index.js";

/**
 * @typedef {import("../core/index.js").SurfaceClass} SurfaceClass
 * @typedef {import("../core/index.js").Vector3} Vector3
 * @typedef {{massKg: number, diameterM: number, radiusM: number, areaM2: number, inertiaKgM2: number, aerodynamicProfile: string}} BallProperties
 * @typedef {{temperatureKelvin: number, pressurePascals?: number, altitudeMetres?: number, relativeHumidity: number}} Atmosphere
 * @typedef {{referenceWindMps: Vector3, verticalMeanWindMps: number, referenceHeightM: number, roughnessLengthM: number, zeroPlaneDisplacementM: number, gustCorrelationTimeS: number, gustSigmaMps: Vector3, gustSeed: number | bigint}} Wind
 * @typedef {{firmness: number, wetness: number, grassLengthIndex: number}} SurfaceCondition
 * @typedef {{sample(x: number, z: number): {heightM: number, dhDx?: number, dhDz?: number, classification: SurfaceClass, condition: SurfaceCondition}}} TerrainProvider
 * @typedef {{ball: BallProperties, air: Atmosphere, windModel: Wind, terrainModel: TerrainProvider}} World
 * @typedef {{maxDepthM: number, featureSizeM: number, seed: number | bigint}} SimplexHeightModifierOptions
 */

const simplexF2 = 0.36602540378443864676;
const simplexG2 = 0.21132486540518711775;
const terrainGradientDeltaM = 0.01;
const uint64Mask = 0xffffffffffffffffn;
const simplexGradients = Object.freeze([
  [1, 1],
  [-1, 1],
  [1, -1],
  [-1, -1],
  [1, 0],
  [-1, 0],
  [1, 0],
  [-1, 0],
  [0, 1],
  [0, -1],
  [0, 1],
  [0, -1]
]);

function assertFiniteNumber(value, description) {
  if (!Number.isFinite(value)) {
    throw new TypeError(`Expected ${description} to be a finite number.`);
  }
  return value;
}

function normalizeSeed(value) {
  if (typeof value === "bigint") {
    if (value < 0n || value > uint64Mask) {
      throw new TypeError("Expected simplex height modifier seed to fit in an unsigned 64-bit integer.");
    }
    return value;
  }
  if (typeof value !== "number" || !Number.isSafeInteger(value) || value < 0) {
    throw new TypeError("Expected simplex height modifier seed to be a non-negative safe integer or bigint.");
  }
  return BigInt.asUintN(64, BigInt(value));
}

function splitMix64Next(state) {
  let nextState = BigInt.asUintN(64, state + 0x9e3779b97f4a7c15n);
  let value = nextState;
  value = BigInt.asUintN(64, (value ^ (value >> 30n)) * 0xbf58476d1ce4e5b9n);
  value = BigInt.asUintN(64, (value ^ (value >> 27n)) * 0x94d049bb133111ebn);
  value = BigInt.asUintN(64, value ^ (value >> 31n));
  return { state: nextState, value };
}

function makeSimplexPermutation(seed) {
  const permutation = Array.from({ length: 256 }, (_, index) => index);
  let state = seed & uint64Mask;
  for (let index = permutation.length - 1; index > 0; index -= 1) {
    const next = splitMix64Next(state);
    state = next.state;
    const swapIndex = Number(next.value % BigInt(index + 1));
    [permutation[index], permutation[swapIndex]] = [permutation[swapIndex], permutation[index]];
  }
  const doubled = new Array(512);
  for (let index = 0; index < doubled.length; index += 1) {
    doubled[index] = permutation[index & 255];
  }
  return doubled;
}

function positiveModulo(value, modulus) {
  return ((value % modulus) + modulus) % modulus;
}

function simplexContribution(t, gradient, x, z) {
  if (t < 0) {
    return 0;
  }
  const tSquared = t * t;
  return tSquared * tSquared * (gradient[0] * x + gradient[1] * z);
}

function simplexNoise2d(x, z, permutation) {
  const skew = (x + z) * simplexF2;
  const i = Math.floor(x + skew);
  const j = Math.floor(z + skew);
  const unskew = (i + j) * simplexG2;
  const x0 = x - (i - unskew);
  const z0 = z - (j - unskew);
  const i1 = x0 > z0 ? 1 : 0;
  const j1 = x0 > z0 ? 0 : 1;
  const x1 = x0 - i1 + simplexG2;
  const z1 = z0 - j1 + simplexG2;
  const x2 = x0 - 1 + 2 * simplexG2;
  const z2 = z0 - 1 + 2 * simplexG2;
  const ii = positiveModulo(i, 256);
  const jj = positiveModulo(j, 256);
  const gi0 = permutation[ii + permutation[jj]] % 12;
  const gi1 = permutation[ii + i1 + permutation[jj + j1]] % 12;
  const gi2 = permutation[ii + 1 + permutation[jj + 1]] % 12;
  const n0 = simplexContribution(0.5 - x0 * x0 - z0 * z0, simplexGradients[gi0], x0, z0);
  const n1 = simplexContribution(0.5 - x1 * x1 - z1 * z1, simplexGradients[gi1], x1, z1);
  const n2 = simplexContribution(0.5 - x2 * x2 - z2 * z2, simplexGradients[gi2], x2, z2);
  return 70 * (n0 + n1 + n2);
}

function normalizeSimplexDepression(noise) {
  return Math.min(Math.max(0.5 * (noise + 1), 0), 1);
}

/**
 * @returns {BallProperties}
 */
export function defaultBallProperties() {
  return {
    massKg: 0.04593,
    diameterM: 0.04267,
    radiusM: 0.021335,
    areaM2: 1.43e-3,
    inertiaKgM2: 8.36e-6,
    aerodynamicProfile: "generic_tour_smits_smith"
  };
}

/**
 * @returns {Atmosphere}
 */
export function defaultAtmosphere() {
  return {
    temperatureKelvin: 288.15,
    pressurePascals: 101325,
    relativeHumidity: 0.5
  };
}

/**
 * @returns {Wind}
 */
export function defaultWind() {
  return {
    referenceWindMps: { x: 0, y: 0, z: 0 },
    verticalMeanWindMps: 0,
    referenceHeightM: 10,
    roughnessLengthM: 0.03,
    zeroPlaneDisplacementM: 0,
    gustCorrelationTimeS: 1.5,
    gustSigmaMps: { x: 0, y: 0, z: 0 },
    gustSeed: 0
  };
}

/**
 * @param {SurfaceClass} classification
 * @returns {SurfaceCondition}
 */
export function defaultSurfaceCondition(classification) {
  switch (classification) {
    case surfaceClass.green:
      return { firmness: 0.7, wetness: 0.1, grassLengthIndex: 0.0 };
    case surfaceClass.ukGreen:
      return { firmness: 0.7, wetness: 1.0, grassLengthIndex: 0.0 };
    case surfaceClass.fairway:
      return { firmness: 0.5, wetness: 0.0, grassLengthIndex: 0.0 };
    case surfaceClass.ukFairway:
      return { firmness: 0.5, wetness: 0.5, grassLengthIndex: 0.5 };
    case surfaceClass.firstCut:
      return { firmness: 0.5, wetness: 0.05, grassLengthIndex: 0.35 };
    case surfaceClass.rough:
      return { firmness: 0.4, wetness: 0.1, grassLengthIndex: 0.8 };
    case surfaceClass.bareSoil:
      return { firmness: 0.8, wetness: 0.05, grassLengthIndex: 0.0 };
    case surfaceClass.wetTurf:
      return { firmness: 0.25, wetness: 1.0, grassLengthIndex: 0.2 };
    case surfaceClass.sand:
      return { firmness: 0.2, wetness: 0.3, grassLengthIndex: 0.0 };
    case surfaceClass.cartPath:
      return { firmness: 0.95, wetness: 0.0, grassLengthIndex: 0.0 };
    default:
      return { firmness: 0.5, wetness: 0.0, grassLengthIndex: 0.0 };
  }
}

/**
 * @param {number} heightM
 * @param {SurfaceClass} classification
 * @param {SurfaceCondition} condition
 * @returns {TerrainProvider}
 */
export function makeFlatTerrain(heightM, classification, condition) {
  return Object.freeze({
    sample() {
      return {
        heightM,
        dhDx: 0,
        dhDz: 0,
        classification,
        condition: { ...condition }
      };
    }
  });
}

/**
 * Wraps a terrain provider with deterministic stackable simplex-noise depressions.
 *
 * @param {TerrainProvider} baseTerrain
 * @param {SimplexHeightModifierOptions} options
 * @returns {TerrainProvider}
 */
export function makeSimplexHeightModifierTerrain(baseTerrain, options) {
  if (typeof baseTerrain !== "object" || baseTerrain === null || typeof baseTerrain.sample !== "function") {
    throw new TypeError("Expected baseTerrain to expose a sample(x, z) function.");
  }
  if (typeof options !== "object" || options === null) {
    throw new TypeError("Expected simplex height modifier options to be an object.");
  }
  if (!Object.hasOwn(options, "seed")) {
    throw new TypeError("Expected simplex height modifier options to include a seed.");
  }

  const maxDepthM = assertFiniteNumber(options.maxDepthM, "simplexHeightModifier.maxDepthM");
  const featureSizeM = assertFiniteNumber(options.featureSizeM, "simplexHeightModifier.featureSizeM");
  if (maxDepthM < 0) {
    throw new TypeError("Simplex height modifier maxDepthM must be non-negative.");
  }
  if (featureSizeM <= 0) {
    throw new TypeError("Simplex height modifier featureSizeM must be positive.");
  }

  const permutation = makeSimplexPermutation(normalizeSeed(options.seed));
  const modifiedHeight = (x, z) => {
    const baseSample = baseTerrain.sample(x, z);
    const depression = maxDepthM * normalizeSimplexDepression(simplexNoise2d(x / featureSizeM, z / featureSizeM, permutation));
    return baseSample.heightM - depression;
  };

  return Object.freeze({
    sample(x, z) {
      const center = baseTerrain.sample(x, z);
      const heightM = modifiedHeight(x, z);
      const dhDx = (modifiedHeight(x + terrainGradientDeltaM, z) - modifiedHeight(x - terrainGradientDeltaM, z)) /
        (2 * terrainGradientDeltaM);
      const dhDz = (modifiedHeight(x, z + terrainGradientDeltaM) - modifiedHeight(x, z - terrainGradientDeltaM)) /
        (2 * terrainGradientDeltaM);
      return {
        heightM,
        dhDx,
        dhDz,
        classification: center.classification,
        condition: typeof center.condition === "object" && center.condition !== null
          ? { ...center.condition }
          : center.condition
      };
    }
  });
}

/**
 * @returns {World}
 */
export function defaultWorld() {
  const ball = defaultBallProperties();
  return {
    ball,
    air: defaultAtmosphere(),
    windModel: defaultWind(),
    terrainModel: makeFlatTerrain(
      -ball.radiusM,
      surfaceClass.fairway,
      defaultSurfaceCondition(surfaceClass.fairway)
    )
  };
}

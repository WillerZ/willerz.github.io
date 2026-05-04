/**
 * @typedef {{x: number, y: number, z: number}} Vector3
 * @typedef {{x: number, y: number, z: number}} Point3
 * @typedef {{speed: number, direction: Vector3}} TranslationalVelocity
 * @typedef {{angularSpeed: number, axis: Vector3}} RotationalVelocity
 * @typedef {{translational: TranslationalVelocity, rotational: RotationalVelocity}} LaunchState
 * @typedef {"green" | "fairway" | "first_cut" | "rough" | "bare_soil" | "wet_turf" | "sand" | "cart_path" | "uk_green" | "uk_fairway"} SurfaceClass
 */

/**
 * Surface-class constants shared across models.
 */
export const surfaceClass = Object.freeze({
  green: "green",
  fairway: "fairway",
  firstCut: "first_cut",
  rough: "rough",
  bareSoil: "bare_soil",
  wetTurf: "wet_turf",
  sand: "sand",
  cartPath: "cart_path",
  ukGreen: "uk_green",
  ukFairway: "uk_fairway"
});

const surfaceClassSet = new Set(/** @type {SurfaceClass[]} */ (Object.values(surfaceClass)));

/**
 * Returns `true` if a value is a finite vector-like object.
 *
 * @param {unknown} value
 * @returns {value is Vector3}
 */
export function isFiniteVector3(value) {
  if (typeof value !== "object" || value === null) {
    return false;
  }

  const candidate = /** @type {{x?: unknown, y?: unknown, z?: unknown}} */ (value);
  return Number.isFinite(candidate.x) &&
    Number.isFinite(candidate.y) &&
    Number.isFinite(candidate.z);
}

/**
 * @param {Vector3} value
 * @param {Vector3} other
 * @returns {number}
 */
export function dot(value, other) {
  return value.x * other.x + value.y * other.y + value.z * other.z;
}

/**
 * @param {Vector3} value
 * @param {Vector3} other
 * @returns {Vector3}
 */
export function cross(value, other) {
  return {
    x: value.y * other.z - value.z * other.y,
    y: value.z * other.x - value.x * other.z,
    z: value.x * other.y - value.y * other.x
  };
}

/**
 * @param {Vector3} value
 * @param {Vector3} other
 * @returns {Vector3}
 */
export function addVector(value, other) {
  return { x: value.x + other.x, y: value.y + other.y, z: value.z + other.z };
}

/**
 * @param {Vector3} value
 * @param {Vector3} other
 * @returns {Vector3}
 */
export function subtractVector(value, other) {
  return { x: value.x - other.x, y: value.y - other.y, z: value.z - other.z };
}

/**
 * @param {Vector3} value
 * @param {number} scale
 * @returns {Vector3}
 */
export function scaleVector(value, scale) {
  return { x: value.x * scale, y: value.y * scale, z: value.z * scale };
}

/**
 * @param {Vector3} value
 * @returns {number}
 */
export function squaredNorm(value) {
  return dot(value, value);
}

/**
 * @param {Vector3} value
 * @returns {number}
 */
export function norm(value) {
  return Math.sqrt(squaredNorm(value));
}

/**
 * @param {number} value
 * @param {number} lower
 * @param {number} upper
 * @returns {number}
 */
export function clamp(value, lower, upper) {
  return Math.min(Math.max(value, lower), upper);
}

/**
 * Attempts to normalize a direction vector.
 *
 * @param {Vector3} value
 * @returns {Vector3 | null}
 */
export function tryNormalizeUnitVector(value) {
  if (!isFiniteVector3(value)) {
    return null;
  }

  const magnitude = norm(value);
  if (!(magnitude > 1e-9)) {
    return null;
  }

  return scaleVector(value, 1 / magnitude);
}

/**
 * Normalizes a direction vector or throws.
 *
 * @param {Vector3} value
 * @param {string} description
 * @returns {Vector3}
 */
export function normalizeUnitVector(value, description = "direction") {
  const normalized = tryNormalizeUnitVector(value);
  if (normalized === null) {
    throw new TypeError(`Expected ${description} to be a finite non-zero vector.`);
  }
  return normalized;
}

/**
 * Converts repository-facing translational velocity to cartesian vector form.
 *
 * @param {TranslationalVelocity} value
 * @returns {Vector3}
 */
export function translationalVelocityToCartesian(value) {
  if (typeof value !== "object" || value === null || !Number.isFinite(value.speed) || value.speed < 0) {
    throw new TypeError("Expected translational velocity to contain a finite non-negative speed.");
  }
  const direction = normalizeUnitVector(value.direction, "translational direction");
  return scaleVector(direction, value.speed);
}

/**
 * Converts a cartesian velocity vector to repository-facing translational velocity.
 *
 * @param {Vector3} value
 * @returns {TranslationalVelocity}
 */
export function cartesianToTranslationalVelocity(value) {
  const speed = norm(value);
  if (!(speed > 1e-12) || !isFiniteVector3(value)) {
    return {
      speed: 0,
      direction: { x: 0, y: 0, z: 1 }
    };
  }

  return {
    speed,
    direction: scaleVector(value, 1 / speed)
  };
}

/**
 * Converts repository-facing rotational velocity to right-hand-rule vector form.
 *
 * @param {RotationalVelocity} value
 * @returns {Vector3}
 */
export function rotationalVelocityToRightHandRule(value) {
  if (typeof value !== "object" || value === null || !Number.isFinite(value.angularSpeed) || value.angularSpeed < 0) {
    throw new TypeError("Expected rotational velocity to contain a finite non-negative angularSpeed.");
  }
  const axis = normalizeUnitVector(value.axis, "rotational axis");
  return scaleVector(axis, -value.angularSpeed);
}

/**
 * Converts a right-hand-rule angular velocity vector to repository-facing rotational velocity.
 *
 * @param {Vector3} value
 * @returns {RotationalVelocity}
 */
export function rightHandRuleToRotationalVelocity(value) {
  const angularSpeed = norm(value);
  if (!(angularSpeed > 1e-12) || !isFiniteVector3(value)) {
    return {
      angularSpeed: 0,
      axis: { x: 0, y: 1, z: 0 }
    };
  }

  return {
    angularSpeed,
    axis: scaleVector(value, -1 / angularSpeed)
  };
}

/**
 * Returns `true` if a string is a supported surface-class value.
 *
 * @param {unknown} value
 * @returns {value is SurfaceClass}
 */
export function isSurfaceClass(value) {
  return typeof value === "string" && surfaceClassSet.has(/** @type {SurfaceClass} */ (value));
}

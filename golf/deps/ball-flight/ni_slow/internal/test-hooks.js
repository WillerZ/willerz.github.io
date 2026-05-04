import { advanceGroundStep, evaluateSurface, evaluateTerrain, modes, normalizeWorld } from "./simulator.js";
import { simulateInternal } from "./simulator.js";

/**
 * @typedef {import("../index.js").LaunchStateInput} LaunchStateInput
 * @typedef {import("../index.js").WorldInput} WorldInput
 * @typedef {import("../index.js").SimulationResult} SimulationResult
 * @typedef {{disableAerodynamics?: boolean, disableSpinDecay?: boolean}} SimulationOverrides
 */

/**
 * Test-only entry point for diagnostic overrides.
 *
 * @param {LaunchStateInput} launchState
 * @param {WorldInput} world
 * @param {SimulationOverrides} overrides
 * @returns {SimulationResult}
 */
export function simulateWithOverrides(launchState, world, overrides = {}) {
  return simulateInternal(launchState, world, overrides);
}

/**
 * Test-only helper that advances one forced pure-roll step from a flat-ground zero-speed contact state.
 *
 * @param {WorldInput} world
 * @returns {import("../../core/index.js").Vector3}
 */
export function pureRollZeroSpeedVelocityAfterOneStepForTest(world) {
  const environment = normalizeWorld(world);
  const terrain = evaluateTerrain(environment.terrainModel, 0, 0);
  const surface = evaluateSurface(terrain);
  const state = {
    position: { x: 0, y: terrain.heightM + environment.ball.radiusM, z: 0 },
    velocity: { x: 0, y: 0, z: 0 },
    omega: { x: 0, y: 0, z: 0 }
  };
  return advanceGroundStep(state, environment.ball, terrain, surface, modes.pureRoll).velocity;
}

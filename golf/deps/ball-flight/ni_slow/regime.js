/**
 * @typedef {"flight" | "roll" | "stop"} TraceRegime
 */

/**
 * Coarse public trace regimes for `ni_slow` samples.
 *
 * `flight` includes airborne flight and hop phases. `roll` includes every moving
 * sustained ground-contact phase. `stop` marks the final at-rest sample.
 *
 * @type {{readonly flight: "flight", readonly roll: "roll", readonly stop: "stop"}}
 */
export const traceRegime = Object.freeze({
  flight: "flight",
  roll: "roll",
  stop: "stop"
});

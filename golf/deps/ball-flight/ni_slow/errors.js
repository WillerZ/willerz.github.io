export class SimulationError extends Error {
  /**
   * @param {string} message
   * @param {string} code
   */
  constructor(message, code = "simulation_failure") {
    super(message);
    this.name = "SimulationError";
    this.code = code;
  }
}

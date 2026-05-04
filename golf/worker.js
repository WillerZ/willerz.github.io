import {
  defaultBallProperties,
  defaultSurfaceCondition,
  defaultWind,
  makeFlatTerrain,
  simulate
} from "./deps/ball-flight/ni_slow/index.js";
import { surfaceClass } from "./deps/ball-flight/core/index.js";

const speedUnits = Object.freeze({
  mps: (value) => value,
  mph: (value) => value * 0.44704,
  kph: (value) => value / 3.6
});

const angleUnits = Object.freeze({
  deg: (value) => value * Math.PI / 180,
  rad: (value) => value
});

const spinUnits = Object.freeze({
  rpm: (value) => value * 2 * Math.PI / 60,
  hz: (value) => value * 2 * Math.PI,
  rads: (value) => value
});

const temperatureUnits = Object.freeze({
  c: (value) => value + 273.15,
  f: (value) => (value - 32) * 5 / 9 + 273.15,
  k: (value) => value,
  r: (value) => value * 5 / 9
});

const altitudeUnits = Object.freeze({
  m: (value) => value,
  ft: (value) => value * 0.3048
});

const fieldLabels = Object.freeze({
  ballSpeed: "ball speed",
  launchAngle: "launch angle",
  azimuth: "azimuth",
  backspin: "backspin",
  sidespin: "sidespin",
  temperature: "temperature",
  altitude: "altitude",
  humidity: "humidity"
});

function readNumber(form, name) {
  const value = Number(form[name]);
  if (!Number.isFinite(value)) {
    throw new TypeError(`Expected ${fieldLabels[name] ?? name} to be a finite number.`);
  }
  return value;
}

function convertUnit(value, unit, table, description) {
  const converter = table[unit];
  if (typeof converter !== "function") {
    throw new TypeError(`Unsupported ${description} unit.`);
  }
  return converter(value);
}

function buildLaunchState(form) {
  const speedMps = convertUnit(readNumber(form, "ballSpeed"), form.speedUnit, speedUnits, "speed");
  const launchAngleRad = convertUnit(readNumber(form, "launchAngle"), form.angleUnit, angleUnits, "angle");
  const azimuthRad = convertUnit(readNumber(form, "azimuth"), form.angleUnit, angleUnits, "angle");
  const backspinRadS = convertUnit(readNumber(form, "backspin"), form.spinUnit, spinUnits, "spin");
  const sidespinRadS = convertUnit(readNumber(form, "sidespin"), form.spinUnit, spinUnits, "spin");

  if (!(speedMps > 0)) {
    throw new TypeError("Ball speed must be greater than zero.");
  }
  if (backspinRadS < 0) {
    throw new TypeError("Backspin must be zero or greater.");
  }

  const horizontal = Math.cos(launchAngleRad);
  const direction = {
    x: horizontal * Math.sin(azimuthRad),
    y: Math.sin(launchAngleRad),
    z: horizontal * Math.cos(azimuthRad)
  };

  const spinMagnitude = Math.hypot(backspinRadS, sidespinRadS);
  const spinAxis = spinMagnitude > 1e-12
    ? {
        x: backspinRadS / spinMagnitude,
        y: -sidespinRadS / spinMagnitude,
        z: 0
      }
    : { x: 1, y: 0, z: 0 };

  return {
    translational: {
      speed: speedMps,
      direction
    },
    rotational: {
      angularSpeed: spinMagnitude,
      axis: spinAxis
    }
  };
}

function buildWorld(form) {
  const temperatureKelvin = convertUnit(
    readNumber(form, "temperature"),
    form.temperatureUnit,
    temperatureUnits,
    "temperature"
  );
  const altitudeMetres = convertUnit(readNumber(form, "altitude"), form.altitudeUnit, altitudeUnits, "altitude");
  const relativeHumidity = readNumber(form, "humidity") / 100;

  if (!(temperatureKelvin > 0)) {
    throw new TypeError("Temperature must be above absolute zero.");
  }
  if (!(relativeHumidity >= 0 && relativeHumidity <= 1)) {
    throw new TypeError("Humidity must be between 0 and 100 percent.");
  }
  if (!(1 - 0.0065 * altitudeMetres / 288.15 > 0)) {
    throw new TypeError("Altitude is outside the supported atmosphere range.");
  }

  const ball = defaultBallProperties();
  return {
    ball,
    air: {
      temperatureKelvin,
      altitudeMetres,
      relativeHumidity
    },
    windModel: defaultWind(),
    terrainModel: makeFlatTerrain(
      -ball.radiusM,
      surfaceClass.ukFairway,
      defaultSurfaceCondition(surfaceClass.ukFairway)
    )
  };
}

function phaseTrace(result) {
  const impactTime = result.impact.timeSeconds;
  return result.trace.map((sample) => {
    const isCarry = sample.timeSeconds <= impactTime + 1e-8;
    const phase = isCarry ? "carry" : sample.regime === "flight" ? "bounce" : "roll";
    return {
      t: sample.timeSeconds,
      x: sample.position.x,
      y: sample.position.y,
      z: sample.position.z,
      phase
    };
  });
}

function summarize(result) {
  const peakHeightM = result.trace.reduce(
    (peak, sample) => Math.max(peak, sample.position.y),
    0
  );
  const incoming = result.impact.incomingDirection;
  const landingAngleRad = Math.atan2(
    Math.abs(incoming.y),
    Math.hypot(incoming.x, incoming.z)
  );

  return {
    trace: phaseTrace(result),
    highlights: {
      peakHeightM,
      carryDistanceM: Math.hypot(result.impact.point.x, result.impact.point.z),
      totalDistanceM: Math.hypot(result.finalRestPoint.x, result.finalRestPoint.z),
      landingAngleRad,
      offlineCarryM: result.impact.point.x,
      offlineTotalM: result.finalRestPoint.x
    },
    impact: result.impact.point,
    finalRestPoint: result.finalRestPoint
  };
}

self.addEventListener("message", (event) => {
  const { requestId, form } = event.data;
  try {
    const launchState = buildLaunchState(form);
    const world = buildWorld(form);
    const result = simulate(launchState, world);

    self.postMessage({
      requestId,
      ok: true,
      units: {
        speedUnit: form.speedUnit,
        angleUnit: form.angleUnit
      },
      simulation: summarize(result)
    });
  } catch (error) {
    self.postMessage({
      requestId,
      ok: false,
      error: error instanceof Error ? error.message : String(error)
    });
  }
});

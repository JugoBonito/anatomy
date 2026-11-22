import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const KIT = join(dirname(fileURLToPath(import.meta.url)), "..", "kit");
const strip = (file) => readFileSync(join(KIT, file), "utf8").replace(/^export /gm, "");

export function kitScript() {
  return strip("iso-kit.mjs");
}

export function glScript() {
  return strip("gl.mjs");
}

export function turnScript() {
  return strip("turn-core.mjs") + "\n" + strip("turn.mjs").replace(/^import[\s\S]*?;\n?/gm, "");
}

export function fixedScript() {
  return strip("turn-fixed-core.mjs") + "\n" + strip("turn-fixed.mjs").replace(/^import[\s\S]*?;\n?/gm, "") + "\n" + strip("canvas-painter.mjs");
}

export function glSharedScript() {
  return strip("gl-shared.mjs");
}

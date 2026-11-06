/** Signal identity helpers shared by the models and the views. */

const UNSAFE = /[^A-Za-z0-9_.-]/gu;

export function EscapeJoint(name) {
  return String(name ?? '').replace(UNSAFE, (character) => {
    const bytes = new TextEncoder().encode(character);
    return [...bytes].map((byte) => `%${byte.toString(16).toUpperCase().padStart(2, '0')}`).join('');
  });
}

export function SignalId(source, jointName, field) {
  return `${source}/${EscapeJoint(jointName)}/${field}`;
}

export function DecodeSignalId(id) {
  const parts = String(id).split('/');
  if (parts.length !== 3) {
    throw new Error(`not a canonical signal id: ${id}`);
  }
  let joint = '';
  try {
    joint = decodeURIComponent(parts[1]);
  } catch {
    joint = parts[1];
  }
  return { source: parts[0], jointName: joint, field: parts[2] };
}

export function SignalKey(signal) {
  if (signal && typeof signal.id === 'string') {
    return signal.id;
  }
  return SignalId(signal.source, signal.joint_name, signal.field);
}

export function SameAxis(left, right) {
  return Boolean(left) && Boolean(right) &&
    left.quantity === right.quantity && left.unit === right.unit;
}

export function CatalogMap(catalog) {
  const map = new Map();
  for (const signal of Array.isArray(catalog) ? catalog : []) {
    if (signal && typeof signal.id === 'string') {
      map.set(signal.id, signal);
    }
  }
  return map;
}

import { Workbench } from './workbench.js';

async function boot() {
  const workbench = new Workbench({ document });
  globalThis.__workbench = workbench;
  try {
    await workbench.Init();
  } catch (error) {
    workbench.Status(error.message);
  }
}

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', boot, { once: true });
} else {
  boot();
}

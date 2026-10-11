import { BOOK_SEED, widgetSeed } from '../lib/seed';
import { check, type ChapterChecks } from './harness';

/** Properties of the module and of the seed discipline that no chapter owns. */
export const commonChecks: ChapterChecks = (rs) => {
  const a = widgetSeed('w1.1', 0);
  const b = widgetSeed('w1.1', 0);
  const c = widgetSeed('w1.1', 1);
  const d = widgetSeed('w1.2', 0);
  return [
    check('seed: the TypeScript BOOK_SEED is robotics_core::BOOK_SEED', rs.book_seed() === BOOK_SEED, `rust ${rs.book_seed()} vs ts ${BOOK_SEED}`),
    check('seed: widgetSeed is a pure function of (id, re-roll)', a === b),
    check('seed: re-rolls and widget ids give distinct seeds', a !== c && a !== d && c !== d),
    check('seed: seeds are 64-bit values (fit a Rust u64)', a >= 0n && a < 1n << 64n),
  ];
};

import { loadRobotics, type CheckResult } from './harness';
import { commonChecks } from './common';
import { ch01Checks } from './ch01';
import { ch02Checks } from './ch02';
import { ch03Checks } from './ch03';
import { ch05Checks } from './ch05';
import { ch04Checks } from './ch04';
import { ch06Checks } from './ch06';
import { katexChecks } from './katex';

const rs = loadRobotics();
const results: CheckResult[] = [
  ...commonChecks(rs),
  ...ch01Checks(rs),
  ...ch02Checks(rs),
  ...ch03Checks(rs),
  ...ch05Checks(rs),
  ...ch04Checks(rs),
  ...ch06Checks(rs),
  ...katexChecks(),
];

let failed = 0;
for (const r of results) {
  if (!r.pass) failed++;
  console.log(`${r.pass ? 'PASS' : 'FAIL'}  ${r.name}${!r.pass && r.detail ? `\n        ${r.detail}` : ''}`);
}
console.log(`\n${results.length - failed}/${results.length} passed`);
process.exit(failed ? 1 : 0);

import katex from 'katex';
import { katexOptions } from '../lib/katex-macros';
import type { CheckResult } from './harness';

/**
 * The KaTeX configuration must render every environment the chapters use. A frozen macro
 * table once turned every matrix into a red error box while the build still passed, so
 * the environments are rendered here and any `katex-error` fails the suite.
 */
const CASES: [string, string][] = [
  ['pmatrix', String.raw`\begin{pmatrix} a & b \\ c & d \end{pmatrix}`],
  ['bmatrix with macro', String.raw`\mat{K} = \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}`],
  ['cases', String.raw`f(x) = \begin{cases} 1 & x > 0 \\ 0 & \text{otherwise} \end{cases}`],
  ['aligned', String.raw`\begin{aligned} \bel(x_t) &= \eta\, p(z_t \mid x_t) \\ &\quad \times \belbar(x_t) \end{aligned}`],
  ['manifold macros', String.raw`T \in \SEtwo,\quad x \bplus \tau,\quad a \bminus b,\quad \Ad_T,\quad \tau \in \setwo`],
  ['term colors', String.raw`\htmlClass{term-prior}{\bel(x_{t-1})}\,\htmlClass{term-measurement}{p(z_t \mid x_t)}`],
  ['norm and transpose', String.raw`\norm{r}^2 = r\T r`],
];

export function katexChecks(): CheckResult[] {
  return CASES.map(([name, tex]) => {
    try {
      const html = katex.renderToString(tex, { ...katexOptions(), throwOnError: false });
      const bad = html.includes('katex-error');
      return { name: `katex: ${name} renders`, pass: !bad, detail: bad ? 'rendered as katex-error' : undefined };
    } catch (e) {
      return { name: `katex: ${name} renders`, pass: false, detail: String((e as Error).message).slice(0, 120) };
    }
  });
}

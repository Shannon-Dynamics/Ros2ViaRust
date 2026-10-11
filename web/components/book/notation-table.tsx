import katex from 'katex';
import { NOTATION } from '@/lib/notation';
import { katexOptions } from '@/lib/katex-macros';

/** The book-wide notation table, typeset with the same KaTeX configuration as the chapters. */
export function NotationTable() {
  return (
    <div className="not-prose mt-3 overflow-x-auto">
      <table className="w-full min-w-[34rem] border-collapse text-sm">
        <thead>
          <tr className="border-b border-fd-border text-left text-fd-muted-foreground">
            <th className="py-2 pr-4 font-medium">Symbol</th>
            <th className="py-2 pr-4 font-medium">Meaning</th>
            <th className="py-2 font-medium">Rust</th>
          </tr>
        </thead>
        <tbody>
          {NOTATION.map((row) => (
            <tr key={row.symbol} className="border-b border-fd-border/60 align-top">
              <td className="py-2 pr-4" dangerouslySetInnerHTML={{ __html: katex.renderToString(row.symbol, katexOptions()) }} />
              <td className="py-2 pr-4">{row.meaning}</td>
              <td className="py-2 font-mono text-xs">{row.where}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}

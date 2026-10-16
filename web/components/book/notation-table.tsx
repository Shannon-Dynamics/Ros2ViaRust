import katex from 'katex';
import { NOTATION } from '@/lib/notation';
import { katexOptions } from '@/lib/katex-macros';

/**
 * The book-wide notation table, typeset with the same KaTeX configuration as the chapters.
 *
 * Symbols are typeset at build time, so no KaTeX reaches the browser. Server component by
 * construction: it imports KaTeX and must never become a client component.
 */
export function NotationTable() {
  return (
    <div className="my-8 overflow-x-auto">
      <table className="w-full min-w-[34rem] border-collapse text-start">
        <caption className="sr-only">Notation used in the book</caption>
        <thead>
          <tr className="border-b border-fd-border">
            <th scope="col" className="eyebrow w-[9rem] py-2 pe-4 text-start align-bottom">Symbol</th>
            <th scope="col" className="eyebrow py-2 pe-4 text-start align-bottom">Meaning</th>
            <th scope="col" className="eyebrow w-[30%] py-2 text-start align-bottom">Rust</th>
          </tr>
        </thead>
        <tbody>
          {NOTATION.map((row) => (
            <tr key={row.symbol} className="border-b border-fd-border/60 align-baseline last:border-b-0">
              <td className="py-2 pe-4 whitespace-nowrap">
                <span className="text-fd-foreground" dangerouslySetInnerHTML={{ __html: katex.renderToString(row.symbol, katexOptions()) }} />
              </td>
              <td className="py-2 pe-4 font-prose text-[0.95rem] leading-snug text-fd-foreground">{row.meaning}</td>
              <td className="py-2 font-mono text-[0.75rem] leading-snug text-fd-muted-foreground">{row.where}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}

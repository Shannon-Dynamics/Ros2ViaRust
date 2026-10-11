import Link from 'next/link';
import { PHASES } from '@/lib/book-structure';

export default function HomePage() {
  const published = PHASES.filter((p) => p.status === 'published');
  const planned = PHASES.filter((p) => p.status === 'planned');
  return (
    <main className="mx-auto w-full max-w-5xl px-4 py-12 sm:px-6 lg:py-16">
      <p className="mb-3 text-sm font-medium tracking-wide text-fd-primary uppercase">
        Problem-based robotics · Rust · ROS 2
      </p>
      <h1 className="text-4xl leading-tight font-semibold tracking-tight sm:text-5xl">ROS via Rust</h1>
      <p className="mt-5 max-w-2xl text-lg leading-relaxed text-fd-foreground/85">
        One robot, a TurtleBot3 in the Webots simulator, and one rule: each chapter opens with a
        concrete problem the robot has, derives only the theory that problem needs, and then shows the
        Rust that solves it. The widgets in the pages run that same Rust, compiled to WebAssembly.
        Nothing is a re-implementation in JavaScript.
      </p>
      <div className="mt-7 flex flex-wrap gap-3">
        <Link
          href="/chapters/ch01-foundation"
          className="rounded-md bg-fd-primary px-4 py-2 text-sm font-medium text-fd-primary-foreground hover:opacity-90"
        >
          Start with Chapter 1
        </Link>
        <Link href="/chapters" className="rounded-md border border-fd-border px-4 py-2 text-sm font-medium hover:bg-fd-accent">
          Reading guide
        </Link>
      </div>

      <h2 className="mt-14 text-2xl font-semibold tracking-tight">Published</h2>
      <p className="mt-1 text-sm text-fd-muted-foreground">
        A part appears here only once its experiments are finished, with its numbers
        measured and its docs written.
      </p>
      <ul className="mt-5 grid gap-4 sm:grid-cols-2">
        {published.map((p) => (
          <li key={p.phase} className="rounded-lg border border-fd-border bg-fd-card p-5">
            <p className="text-xs font-medium tracking-wide text-fd-muted-foreground uppercase">Part {p.phase}</p>
            <h3 className="mt-1 text-lg font-semibold">{p.title}</h3>
            <p className="mt-2 text-sm text-fd-muted-foreground">{p.scope}</p>
            <ul className="mt-3 space-y-1 text-sm">
              {p.chapters.map((c) => (
                <li key={c.slug}>
                  <Link href={`/chapters/${c.slug}`} className="text-fd-primary hover:underline">
                    Chapter {c.n} · {c.title}
                  </Link>
                </li>
              ))}
            </ul>
          </li>
        ))}
      </ul>

      <h2 className="mt-12 text-2xl font-semibold tracking-tight">Planned</h2>
      <p className="mt-1 text-sm text-fd-muted-foreground">
        Not written yet, and not linked. A part is published once its work is finished and its chapters are written.
      </p>
      <ul className="mt-5 grid gap-3 sm:grid-cols-2 lg:grid-cols-3">
        {planned.map((p) => (
          <li key={p.phase} className="rounded-lg border border-dashed border-fd-border p-4 text-fd-muted-foreground">
            <p className="text-xs font-medium tracking-wide uppercase">Part {p.phase} · planned</p>
            <h3 className="mt-1 font-semibold text-fd-foreground/80">{p.title}</h3>
            <p className="mt-1 text-sm">{p.scope}</p>
          </li>
        ))}
      </ul>
    </main>
  );
}

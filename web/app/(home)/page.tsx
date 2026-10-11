import Link from 'next/link';
import { CHAPTERS, PHASES } from '@/lib/book-structure';

export default function HomePage() {
  const currentParts = PHASES.filter((p) => p.phase <= 4);
  const laterParts = PHASES.filter((p) => p.phase > 4);
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

      <h2 className="mt-14 text-2xl font-semibold tracking-tight">Parts 1–4</h2>
      <p className="mt-1 text-sm text-fd-muted-foreground">
        Part 3 is localization and is planned; Part 4 is mapping and contains Chapter 4.
      </p>
      <ul className="mt-5 grid gap-3 sm:grid-cols-2 lg:grid-cols-4">
        {currentParts.map((part) => (
          <li key={part.phase} className="rounded-lg border border-fd-border bg-fd-card p-4">
            <p className="text-xs font-medium tracking-wide text-fd-muted-foreground uppercase">
              Part {part.phase} · {part.status}
            </p>
            <h3 className="mt-1 font-semibold">{part.title}</h3>
          </li>
        ))}
      </ul>

      <h2 className="mt-12 text-2xl font-semibold tracking-tight">Published chapters</h2>
      <p className="mt-1 text-sm text-fd-muted-foreground">
        Read Chapters 1–4 in order, from foundations through mapping.
      </p>
      <ul className="mt-5 grid gap-4 sm:grid-cols-2">
        {CHAPTERS.map((chapter) => (
          <li key={chapter.slug} className="rounded-lg border border-fd-border bg-fd-card p-5">
            <p className="text-xs font-medium tracking-wide text-fd-muted-foreground uppercase">Chapter {chapter.n}</p>
            <h3 className="mt-1 text-lg font-semibold">
              <Link href={`/chapters/${chapter.slug}`} className="text-fd-primary hover:underline">
                {chapter.title}
              </Link>
            </h3>
          </li>
        ))}
      </ul>

      <h2 className="mt-12 text-2xl font-semibold tracking-tight">Later parts</h2>
      <p className="mt-1 text-sm text-fd-muted-foreground">
        Planned parts have no chapter link until their chapters are written.
      </p>
      <ul className="mt-5 grid gap-3 sm:grid-cols-2 lg:grid-cols-3">
        {laterParts.map((p) => (
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

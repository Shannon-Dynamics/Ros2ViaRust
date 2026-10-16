import Link from 'next/link';
import { CHAPTERS, PHASES } from '@/lib/book-structure';

/** The landing page. Chrome follows the reference's front door (design/STYLE_PARITY.md §5); the words are this book's. */
export default function HomePage() {
  const currentParts = PHASES.filter((p) => p.phase <= 5);
  const laterParts = PHASES.filter((p) => p.phase > 5);
  return (
    <main className="mx-auto w-full max-w-6xl px-4 py-12 sm:px-6 lg:py-16">
      <section className="max-w-3xl">
        <p className="eyebrow mb-4">Problem-based robotics · Rust · ROS 2</p>
        <h1 className="font-display text-4xl leading-[1.08] font-semibold tracking-tight sm:text-5xl">
          ROS <span className="block text-fd-muted-foreground">via Rust</span>
        </h1>
        <p className="mt-5 max-w-xl font-prose text-lg leading-relaxed text-fd-foreground/85">
          One robot, a TurtleBot3 in the Webots simulator, and one rule: each chapter opens with a
          concrete problem the robot has, derives only the theory that problem needs, and then shows the
          Rust that solves it. The widgets in the pages run that same Rust, compiled to WebAssembly.
          Nothing is a re-implementation in JavaScript.
        </p>
        <div className="mt-7 flex flex-wrap items-center gap-3">
          <Link
            href="/chapters/ch01-foundation"
            className="rounded-md bg-fd-primary px-4 py-2 font-ui text-sm font-medium text-fd-primary-foreground transition-opacity hover:opacity-90"
          >
            Start with Chapter 1
          </Link>
          <Link
            href="/chapters"
            className="rounded-md border border-fd-border px-4 py-2 font-ui text-sm font-medium transition-colors hover:bg-fd-accent"
          >
            Reading guide
          </Link>
        </div>
      </section>

      <section className="mt-20 border-t border-fd-border pt-10">
        <h2 className="font-display text-2xl font-semibold">What the chapters cover</h2>
        <p className="mt-1 font-prose text-sm leading-relaxed text-fd-muted-foreground">
          Five topics are published: localization holds Chapter 4 (the EKF half), mapping holds Chapter 5, SLAM holds Chapter 6.
        </p>
        <ul
          role="list"
          className="mt-6 grid gap-px overflow-hidden rounded-lg border border-fd-border bg-fd-border sm:grid-cols-2 lg:grid-cols-5"
        >
          {currentParts.map((part) => (
            <li key={part.phase} className="bg-fd-card p-5">
              <p className="font-mono text-[0.7rem] tracking-widest text-fd-primary uppercase">{part.status}</p>
              <h3 className="mt-1 font-display text-base font-semibold">{part.title}</h3>
            </li>
          ))}
        </ul>
      </section>

      <section className="mt-16">
        <h2 className="font-display text-2xl font-semibold">Published chapters</h2>
        <p className="mt-1 font-prose text-sm leading-relaxed text-fd-muted-foreground">
          Read Chapters 1 to 6 in order: 1, 2, 3, 4 (localization), 5 (mapping), 6 (SLAM).
        </p>
        <ul role="list" className="mt-4">
          {CHAPTERS.map((chapter) => (
            <li key={chapter.slug} className="border-b border-fd-border/60 last:border-0">
              <Link
                href={`/chapters/${chapter.slug}`}
                className="group grid grid-cols-[2rem_1fr] gap-3 py-2.5 transition-colors hover:bg-fd-accent/40"
              >
                <span className="pt-0.5 font-mono text-xs text-fd-muted-foreground tabular-nums">
                  {String(chapter.n).padStart(2, '0')}
                </span>
                <span className="font-ui text-sm font-medium group-hover:text-fd-primary">{chapter.title}</span>
              </Link>
            </li>
          ))}
        </ul>
      </section>

      <section className="mt-16">
        <h2 className="font-display text-2xl font-semibold">Later topics</h2>
        <p className="mt-1 font-prose text-sm leading-relaxed text-fd-muted-foreground">
          Planned topics have no chapter link until their chapters are written.
        </p>
        <ul role="list" className="mt-6 grid gap-3 sm:grid-cols-2 lg:grid-cols-3">
          {laterParts.map((p) => (
            <li key={p.phase} className="rounded-lg border border-dashed border-fd-border p-4 text-fd-muted-foreground">
              <p className="font-mono text-[0.7rem] tracking-widest uppercase">planned</p>
              <h3 className="mt-1 font-display text-base font-semibold text-fd-foreground/80">{p.title}</h3>
              <p className="mt-1 font-prose text-sm leading-snug">{p.scope}</p>
            </li>
          ))}
        </ul>
      </section>
    </main>
  );
}

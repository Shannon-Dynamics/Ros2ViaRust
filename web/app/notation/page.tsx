import { HomeLayout } from 'fumadocs-ui/layouts/home';
import { bookNav } from '@/lib/nav';
import { NotationTable } from '@/components/book/notation-table';
import { ColorKey } from '@/components/book/color-key';

export const metadata = { title: 'Notation' };

export default function NotationPage() {
  return (
    <HomeLayout {...bookNav}>
      <main className="mx-auto w-full max-w-4xl px-4 py-12 sm:px-6">
        <h1 className="text-3xl font-semibold tracking-tight">Notation and color code</h1>
        <p className="mt-3 text-fd-muted-foreground">
          One table for the whole book. A chapter may add symbols to its own table; it may not change
          one of these.
        </p>
        <h2 className="mt-8 text-xl font-semibold">Symbols</h2>
        <NotationTable />
        <h2 className="mt-10 text-xl font-semibold">The five roles</h2>
        <p className="mt-2 text-fd-muted-foreground">
          Every figure, equation term, and widget uses the same five colors for the same five things.
        </p>
        <ColorKey roles={['prior', 'prediction', 'measurement', 'posterior', 'truth']} className="mt-4" />
      </main>
    </HomeLayout>
  );
}

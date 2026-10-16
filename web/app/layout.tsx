import './global.css';
import type { Metadata } from 'next';
import { Fraunces, IBM_Plex_Mono, IBM_Plex_Sans, Source_Serif_4 } from 'next/font/google';
import { RootProvider } from 'fumadocs-ui/provider/next';

// The four faces and the weights the stylesheet uses (design/STYLE_PARITY.md §2). The font
// variables go on <html>, not <body>: Tailwind's `@theme` declares --font-display and friends on
// :root, so a variable defined only further down the tree resolves to its fallback there and the
// face never applies through `font-display` / `font-prose`.
const fraunces = Fraunces({
  subsets: ['latin'],
  display: 'swap',
  axes: ['SOFT', 'WONK'],
  variable: '--font-fraunces',
});

const sourceSerif = Source_Serif_4({
  subsets: ['latin'],
  display: 'swap',
  variable: '--font-source-serif',
});

const plexSans = IBM_Plex_Sans({
  subsets: ['latin'],
  weight: ['400', '500', '600'],
  display: 'swap',
  variable: '--font-plex-sans',
});

const plexMono = IBM_Plex_Mono({
  subsets: ['latin'],
  weight: ['400', '500'],
  display: 'swap',
  variable: '--font-plex-mono',
});

export const metadata: Metadata = {
  title: { default: 'ROS via Rust', template: '%s · ROS via Rust' },
  description:
    'Problem-based robotics in Rust: each chapter opens with a problem a TurtleBot3 has, derives the theory it needs, and runs the real Rust, compiled to WebAssembly, in the page.',
};

export default function RootLayout({ children }: { children: React.ReactNode }) {
  return (
    <html
      lang="en"
      className={`${fraunces.variable} ${sourceSerif.variable} ${plexSans.variable} ${plexMono.variable}`}
      suppressHydrationWarning
    >
      <body className="flex min-h-screen flex-col">
        <RootProvider>{children}</RootProvider>
      </body>
    </html>
  );
}

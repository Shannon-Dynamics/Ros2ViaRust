import type { BaseLayoutProps } from 'fumadocs-ui/layouts/shared';

/** Chrome shared by the landing page and the chapter reader. */
export const bookNav: BaseLayoutProps = {
  nav: {
    title: (
      <span className="flex items-center gap-2">
        <svg width="20" height="20" viewBox="0 0 20 20" fill="none" aria-hidden="true" className="text-fd-primary">
          {/* A pose estimate inside its uncertainty ellipse, the book's recurring picture. */}
          <ellipse cx="10" cy="10" rx="8.2" ry="4.6" transform="rotate(-28 10 10)" stroke="currentColor" strokeWidth="1.3" opacity="0.55" />
          <ellipse cx="10" cy="10" rx="4.6" ry="2.4" transform="rotate(-28 10 10)" stroke="currentColor" strokeWidth="1.3" />
          <circle cx="10" cy="10" r="1.7" fill="currentColor" />
        </svg>
        <span className="font-display text-[0.95rem] font-semibold tracking-tight">
          ROS <span className="text-fd-muted-foreground">via Rust</span>
        </span>
      </span>
    ),
    url: '/',
  },
  links: [
    { text: 'Chapters', url: '/chapters', active: 'nested-url' },
    { text: 'Notation', url: '/notation' },
  ],
};

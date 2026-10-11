import defaultComponents from 'fumadocs-ui/mdx';
import type { MDXComponents } from 'mdx/types';
import { Callout } from 'fumadocs-ui/components/callout';
import { Tab, Tabs } from 'fumadocs-ui/components/tabs';

import { Overview } from '@/components/book/overview';
import { Epigraph } from '@/components/book/epigraph';
import { Derivation } from '@/components/book/derivation';
import { Algorithm } from '@/components/book/algorithm';
import { Exercises, Exercise } from '@/components/book/exercises';
import { References, Reference } from '@/components/book/references';
import { KeyIdea } from '@/components/book/key-idea';
import { ColorKey } from '@/components/book/color-key';
import { RoleTag } from '@/components/book/role-tag';
import { Predict } from '@/components/book/predict';
import { WidgetFrame } from '@/components/sim/widget-frame';

/** Everything a chapter's MDX may use without importing. Widgets are imported by the chapter. */
export function getMDXComponents(components?: MDXComponents): MDXComponents {
  return {
    ...defaultComponents,
    Callout,
    Tab,
    Tabs,
    Overview,
    Epigraph,
    Derivation,
    Algorithm,
    Exercises,
    Exercise,
    References,
    Reference,
    KeyIdea,
    ColorKey,
    RoleTag,
    Predict,
    WidgetFrame,
    ...components,
  };
}

export const useMDXComponents = getMDXComponents;

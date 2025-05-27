import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'FEAGI',
  tagline: 'Flexible & Extensible Artificial General Intelligence',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://feagi.org',
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/',

  // GitHub pages deployment config.
  organizationName: 'feagi',
  projectName: 'feagi',

  onBrokenLinks: 'log', // Changed from warn to log to allow development
  onBrokenMarkdownLinks: 'log',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  plugins: [
    // System documentation - main /docs folder
    ['@docusaurus/plugin-content-docs', {
      id: 'system',
      path: '../../docs',
      routeBasePath: 'system',
      sidebarPath: './sidebars/systemSidebar.ts',
      exclude: [
        'plan-documentation-restructuring-progress.md',
        '**/archive/**',
      ],
    }],

    // API module documentation
    ['@docusaurus/plugin-content-docs', {
      id: 'api',
      path: '../../feagi/api',
      routeBasePath: 'modules/api',
      sidebarPath: './sidebars/apiSidebar.ts',
      include: ['**/*.md', '**/*.mdx'],
      exclude: [
        'protocols/byte_structures/*.md',
        'protocols/byte_structures/README.md',
        'protocols/byte_structures/008 - Single Raw Image.md',
        'protocols/byte_structures/009 - Multi bytestruct holder.md',
        'protocols/byte_structures/011 - Neuron Potential Data (Categories, XYZ).md',
        'protocols/README.md',
      ],
    }],

    // BDU module documentation
    ['@docusaurus/plugin-content-docs', {
      id: 'bdu',
      path: '../../feagi/bdu',
      routeBasePath: 'modules/bdu',
      sidebarPath: './sidebars/bduSidebar.ts',
      include: ['**/*.md', '**/*.mdx'],
      exclude: [
        'embryogenesis/arch-neuroembryogenesis.md',
        'arch-bdu.md',
      ],
    }],

    // NPU module documentation
    ['@docusaurus/plugin-content-docs', {
      id: 'npu',
      path: '../../feagi/npu',
      routeBasePath: 'modules/npu',
      sidebarPath: './sidebars/npuSidebar.ts',
      include: ['**/*.md', '**/*.mdx'],
      exclude: [
        'fcl_example.md',
        'burst_engine.md',
      ],
    }],

    // Core module documentation
    ['@docusaurus/plugin-content-docs', {
      id: 'core',
      path: '../../feagi/core',
      routeBasePath: 'modules/core',
      sidebarPath: './sidebars/coreSidebar.ts',
      include: ['**/*.md', '**/*.mdx'],
    }],

    // Evolution module documentation
    ['@docusaurus/plugin-content-docs', {
      id: 'evo',
      path: '../../feagi/evo',
      routeBasePath: 'modules/evo',
      sidebarPath: './sidebars/evoSidebar.ts',
      include: ['**/*.md', '**/*.mdx'],
      exclude: [
        'README.md',
      ],
    }],

    // PNS module documentation
    ['@docusaurus/plugin-content-docs', {
      id: 'pns',
      path: '../../feagi/pns',
      routeBasePath: 'modules/pns',
      sidebarPath: './sidebars/pnsSidebar.ts',
      include: ['**/*.md', '**/*.mdx'],
    }],


  ],

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars/userGuideSidebar.ts',
          routeBasePath: '/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/feagi-social-card.jpg',
    navbar: {
      title: 'FEAGI',
      logo: {
        alt: 'FEAGI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          to: '/',
          position: 'left',
          label: 'User Guides',
        },
        {
          to: '/system/arch-system-overview',
          position: 'left',
          label: 'System Documentation',
        },
        {
          position: 'left',
          label: 'Module Documentation',
          items: [
            {
              label: 'API',
              to: '/modules/api',
            },
            {
              label: 'BDU',
              to: '/modules/bdu',
            },
            {
              label: 'NPU',
              to: '/modules/npu',
            },
            {
              label: 'Core',
              to: '/modules/core',
            },
            {
              label: 'Evolution',
              to: '/modules/evo',
            },
            {
              label: 'PNS',
              to: '/modules/pns',
            },
          ],
        },
        {
          href: 'https://github.com/feagi/feagi',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Documentation',
          items: [
            {
              label: 'User Guides',
              to: '/',
            },
            {
              label: 'System Documentation',
              to: '/system/arch-system-overview',
            },
            {
              label: 'API Documentation',
              to: '/modules/api',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Discord',
              href: 'https://discord.gg/feagi',
            },
            {
              label: 'Twitter',
              href: 'https://twitter.com/feagi',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/feagi/feagi',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} FEAGI Project. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;

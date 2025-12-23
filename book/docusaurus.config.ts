import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics: A Technical Book',
  tagline: 'Bridging AI Brains with Physical Bodies through ROS 2, Gazebo, Unity, and Isaac Sim',
  favicon: '/img/favicon.ico',

  future: {
    v4: true,
  },

  // ✅ Vercel domain (replace with your exact Vercel link)
  url: 'https://hackathon-l-physical-ai-and-humanoi.vercel.app',

  // ❗ REQUIRED for Vercel – Always "/"
  baseUrl: '/',

  // Safe to keep
  organizationName: 'Wajihakamran-14',
  projectName: 'Hackathon-l-Physical-AI-and-Humanoid-Robotics-Text-Book',

  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl:
            'https://github.com/Wajihakamran-14/Hackathon-l-Physical-AI-and-Humanoid-Robotics-Text-Book/tree/main/book/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: '/img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'bookSidebar',
          position: 'left',
          label: 'Book',
        },
        {
          href: 'https://github.com/Wajihakamran-14/Hackathon-l-Physical-AI-and-Humanoid-Robotics-Text-Book/',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Book Content',
          items: [
            {
              label: 'Table of Contents',
              to: '/docs/intro',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub Discussions',
              href: 'https://github.com/Wajihakamran-14/Hackathon-l-Physical-AI-and-Humanoid-Robotics-Text-Book/discussions',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub Repository',
              href: 'https://github.com/Wajihakamran-14/Hackathon-l-Physical-AI-and-Humanoid-Robotics-Text-Book/',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Project. Built with 💚 by Wajiha Kamran.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;


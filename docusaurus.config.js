// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From ROS 2 to Isaac Sim: The Future of Embodied Intelligence.',
  url: 'https://physical-ai-book.vercel.app',
  baseUrl: '/',
  
  // FIX 1: Prevent Vercel Crash
  onBrokenLinks: 'warn', 
  onBrokenMarkdownLinks: 'warn',
  
  favicon: 'img/favicon.ico',

  // GitHub pages deployment config.
  organizationName: 'mansoorahmedsurti', 
  projectName: 'Physical-AI-and-Robotics-Book',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          routeBasePath: 'docs',
        },
        blog: false, 
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Robotics Logo',
          src: 'main_logo.png',
        },
        items: [
          {
            type: 'doc',
            docId: 'intro',
            position: 'left',
            label: 'Start Reading',
          },
          {
            href: 'https://github.com/mansoorahmedsurti/Physical-AI-and-Robotics-Book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Start Reading',
                to: '/docs/intro',
              },
            ],
          },
        ],
        // FIX 2: Clean Copyright Line
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
      },
    }),
};

module.exports = config;
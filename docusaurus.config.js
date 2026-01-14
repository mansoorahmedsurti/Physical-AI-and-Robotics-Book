// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From ROS 2 to Isaac Sim: The Future of Embodied Intelligence.',
  url: 'https://robotics-book-by-mansoor.vercel.app',
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

  plugins: [
    // No redirects needed since we have a floating chat widget
  ],

  // Proxy configuration for API requests
  themes: [
    // Add proxy configuration
  ],

  // Custom fields for API configuration
  customFields: {
    apiUrl: process.env.API_URL || 'https://mansoorahmedsurti-rag-chatbot-robotics.hf.space',
  },

themeConfig:

    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Robotics Logo',
          src: 'img/main_logo.png',
        },
        items: [
          {
            href: 'https://github.com/mansoorahmedsurti/Physical-AI-and-Robotics-Book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        // Removed 'Docs' link item
        links: [],
        // FIX 2: Clean Copyright Line
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus by <a href="https://mansoor--ahmed.vercel.app/" target="_blank" rel="noopener noreferrer">Mansoor Ahmed</a>.`,
      },
      metadata: [
        {name: 'keywords', content: 'robotics, AI, machine learning, ROS, Isaac Sim, humanoid robotics, physical AI, embodied intelligence'},
        {name: 'author', content: 'Mansoor Ahmed'},
        {property: 'og:title', content: 'Physical AI & Humanoid Robotics'},
        {property: 'og:description', content: 'From ROS 2 to Isaac Sim: The Future of Embodied Intelligence'},
        {property: 'og:image', content: 'https://robotics-book-by-mansoor.vercel.app/img/og-image.jpg'}, // Update with actual image path
        {property: 'og:url', content: 'https://robotics-book-by-mansoor.vercel.app'},
        {property: 'og:type', content: 'website'},
        {name: 'twitter:card', content: 'summary_large_image'},
        {name: 'twitter:title', content: 'Physical AI & Humanoid Robotics'},
        {name: 'twitter:description', content: 'From ROS 2 to Isaac Sim: The Future of Embodied Intelligence'},
        {name: 'twitter:image', content: 'https://robotics-book-by-mansoor.vercel.app/img/og-image.jpg'}, // Update with actual image path
      ],
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
        additionalLanguages: ['python', 'bash', 'json', 'yaml', 'docker'],
      },
    }),
};

module.exports = config;


module.exports = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From ROS 2 to Isaac Sim: The Future of Embodied Intelligence.',
  url: 'https://your-domain.com', // Replace with your actual domain
  baseUrl: '/',
  favicon: 'img/favicon.ico',

  organizationName: 'your-org', // Replace with your GitHub organization name
  projectName: 'your-repo',   // Replace with your GitHub repository name

  themeConfig: {
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Site Logo',
        src: 'img/logo.svg', // Placeholder logo path, replace if you have one
      },
      items: [
        {
          type: 'doc',
          docId: 'intro',
          position: 'left',
          label: 'Start Reading',
        },
        {
          type: 'doc',
          // This assumes you have a category named 'curriculum'.
          // If not, adjust this ID or ensure the category exists.
          docId: 'category/curriculum',
          position: 'left',
          label: 'Curriculum',
        },
        {
          href: 'https://github.com/your-org/your-repo', // Replace with your actual GitHub repo URL
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
            {
              label: 'Curriculum',
              to: '/docs/category/curriculum', // Placeholder
            },
          ],
        },
        // Add more footer links if needed
      ],
      copyright: \`Copyright © \${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.\`,
    },
  },

  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl:
            'https://github.com/your-org/your-repo/edit/main/docs/', // Update to your repo's edit URL
          routeBasePath: 'docs',
        },
        blog: {
          showReadingTime: true,
          editUrl:
            'https://github.com/your-org/your-repo/edit/main/blog/', // Update for blog if applicable
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],
};

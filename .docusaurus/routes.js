import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '00f'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'acc'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '8c6'),
            routes: [
              {
                path: '/docs/Capstone_The_Autonomous_Humanoid/',
                component: ComponentCreator('/docs/Capstone_The_Autonomous_Humanoid/', '485'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/Capstone_The_Autonomous_Humanoid/docs/API_DOCS',
                component: ComponentCreator('/docs/Capstone_The_Autonomous_Humanoid/docs/API_DOCS', '2de'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '2b5'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/Module_1_The_Robotic_Nervous_System_ROS_2/',
                component: ComponentCreator('/docs/Module_1_The_Robotic_Nervous_System_ROS_2/', '61b'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/Module_2_The_Digital_Twin_Gazebo_Unity/',
                component: ComponentCreator('/docs/Module_2_The_Digital_Twin_Gazebo_Unity/', '3b2'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/Module_3_The_AI_Robot_Brain_NVIDIA_Isaac/',
                component: ComponentCreator('/docs/Module_3_The_AI_Robot_Brain_NVIDIA_Isaac/', '6c9'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/Module_4_Vision_Language_Action_VLA/',
                component: ComponentCreator('/docs/Module_4_Vision_Language_Action_VLA/', '292'),
                exact: true,
                sidebar: "docs"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];

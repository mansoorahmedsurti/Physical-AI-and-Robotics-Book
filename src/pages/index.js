import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">Physical AI & Humanoid Robotics</h1>
        <p className="hero__subtitle">From ROS 2 to Isaac Sim: The Future of Embodied Intelligence</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning
          </Link>
        </div>
      </div>
    </header>
  );
}

function FeatureColumn({title, description}) {
  return (
    <div className={clsx('col col--4 text--center padding-horiz--md')}>
      <h3>{title}</h3>
      <p>{description}</p>
    </div>
  );
}

function Homepage() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home - ${siteConfig.title}`}
      description="The official documentation for Physical AI & Humanoid Robotics.">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <FeatureColumn
                title="Robotic Operating System"
                description="Deep dive into ROS 2 nodes and communication."
              />
              <FeatureColumn
                title="Physical Simulation"
                description="Training agents in NVIDIA Isaac Sim."
              />
              <FeatureColumn
                title="VLA Models"
                description="Integration of Vision-Language-Action models."
              />
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}

export default Homepage;

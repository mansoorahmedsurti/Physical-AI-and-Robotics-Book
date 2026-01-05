import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className="hero hero--primary hero--centered" style={{ padding: '4rem 0' }}>
      <div className="container">
        <h1 className="hero__title" style={{ fontSize: '5rem', background: 'linear-gradient(45deg, #3b82f6, #1e3a8a)', WebkitBackgroundClip: 'text', WebkitTextFillColor: 'transparent', backgroundClip: 'text', color: 'transparent', marginBottom: '1rem' }}>
          Physical AI & Humanoid Robotics
        </h1>
        <p className="hero__subtitle" style={{ color: '#d1d5db', fontSize: '1.25rem', marginBottom: '2rem' }}>
          From ROS 2 to Isaac Sim: The Future of Embodied Intelligence
        </p>
        <div className="buttons" style={{ display: 'flex', justifyContent: 'center', gap: '1rem' }}>
          <Link
            className="button button--primary button--lg"
            to="/docs/intro">
            Start Learning
          </Link>
          <Link
            className="button button--secondary button--lg"
            to="https://github.com/anthropics/claude-code">
            View on GitHub
          </Link>
        </div>
      </div>
    </header>
  );
}

function FeatureColumn({title, description}) {
  return (
    <div className='col col--4 text--center padding-horiz--md card'>
      <h3 style={{ color: '#3b82f6', marginBottom: '0.75rem' }}>{title}</h3>
      <p style={{ color: '#e5e7eb' }}>{description}</p>
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
        <section style={{ padding: '2rem 0' }}>
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

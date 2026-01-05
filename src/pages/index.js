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
            Get Started
          </Link>
          <Link
            className="button button--secondary button--lg" style={{ color: 'white' }}
            to="https://github.com/mansoorahmedsurti/Physical-AI-and-Robotics-Book">
            View on GitHub
          </Link>
        </div>
      </div>
    </header>
  );
}

function FeatureColumn({title, description}) {
  return (
    <div className='col col--4 text--center padding-horiz--md card' style={{ margin: '1rem' }}>
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
        <div className="container" style={{ padding: '4rem 0' }}>
          {/* CRITICAL: We change 'row' to 'featuresGrid' to activate your CSS */}
          <div className="featuresGrid">
            
            {/* Card 1 */}
            <div className="card">
              <h3>Robotic Operating System</h3>
              <p>Deep dive into ROS 2 nodes and communication.</p>
            </div>

            {/* Card 2 */}
            <div className="card">
              <h3>Physical Simulation</h3>
              <p>Training agents in NVIDIA Isaac Sim.</p>
            </div>

            {/* Card 3 */}
            <div className="card">
              <h3>VLA Models</h3>
              <p>Integration of Vision-Language-Action models.</p>
            </div>
            {/* --- NEW ROW --- */}

            {/* Card 4: Reinforcement Learning */}
            <div className="card">
              <h3>Gym & RL Training</h3>
              <p>
                Train robust walking policies using <strong>OmniIsaacGymEnvs</strong> and PPO/SAC algorithms.
              </p>
            </div>

            {/* Card 5: Sim2Real Transfer */}
            <div className="card">
              <h3>Sim2Real Gap</h3>
              <p>
                Bridge the reality gap. Techniques to deploy simulation policies onto real <strong>physical hardware</strong>.
              </p>
            </div>

            {/* Card 6: Edge Computing */}
            <div className="card">
              <h3>Edge Deployment</h3>
              <p>
                Optimize your nodes for <strong>NVIDIA Jetson Orin</strong>. Run heavy VLA models with low latency.
              </p>
            </div>

          </div>
        </div>
      </main>
    </Layout>
  );
}

export default Homepage;

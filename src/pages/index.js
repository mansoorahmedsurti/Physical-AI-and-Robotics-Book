import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import TypewriterEffect from '../components/TypewriterEffect';
import SpotlightCard from '../components/SpotlightCard';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className="hero hero--primary hero--centered">
      <div className="container">
        <h1 className="hero__title">
          Physical AI & Humanoid Robotics
        </h1>
        <div className="hero__subtitle">
          <TypewriterEffect />
        </div>
        <div className="buttons">
          <Link
            className="button button--primary button--lg"
            to="/docs/intro">
            Get Started
          </Link>
          <Link
            className="button button--secondary button--lg"
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
    <div className='col col--4 text--center padding-horiz--md card'>
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
        <div className="container padding-vert--lg">
          {/* CRITICAL: We change 'row' to 'featuresGrid' to activate your CSS */}
          <div className="featuresGrid">

            {/* Card 1: Edge Deployment */}
            <SpotlightCard
              title="Edge Deployment"
              description="Optimize your nodes for NVIDIA Jetson Orin. Run heavy VLA models with low latency."
            >
              <p>
                Optimize your nodes for <strong>NVIDIA Jetson Orin</strong>. Run heavy VLA models with low latency.
              </p>
            </SpotlightCard>

            {/* Card 2: Robotic Operating System */}
            <SpotlightCard
              title="Robotic Operating System"
              description="Deep dive into ROS 2 nodes and communication."
            >
              <p>Deep dive into <strong>ROS 2</strong> nodes and communication.</p>
            </SpotlightCard>

            {/* Card 3: Sim2Real Gap */}
            <SpotlightCard
              title="Sim2Real Gap"
              description="Bridge the reality gap. Techniques to deploy simulation policies onto real physical hardware."
            >
              <p>
                Bridge the reality gap. Techniques to deploy simulation policies onto real <strong>physical hardware</strong>.
              </p>
            </SpotlightCard>
            {/* --- NEW ROW --- */}

            {/* Card 4: VLA Models */}
            <SpotlightCard
              title="VLA Models"
              description="Integration of Vision-Language-Action models."
            >
              <p>Integration of <strong>Vision-Language-Action</strong> models.</p>
            </SpotlightCard>

            {/* Card 5: Physical Simulation */}
            <SpotlightCard
              title="Physical Simulation"
              description="Training agents in NVIDIA Isaac Sim."
            >
              <p>Training agents in <strong>NVIDIA Isaac Sim</strong>.</p>
            </SpotlightCard>

            {/* Card 6: Gym & RL Training */}
            <SpotlightCard
              title="Gym & RL Training"
              description="Train robust walking policies using OmniIsaacGymEnvs and PPO/SAC algorithms."
            >
              <p>
                Train robust walking policies using <strong>OmniIsaacGymEnvs</strong> and PPO/SAC algorithms.
              </p>
            </SpotlightCard>

          </div>
        </div>
      </main>
    </Layout>
  );
}

export default Homepage;

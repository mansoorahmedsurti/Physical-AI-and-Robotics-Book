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
      <div className="video-background">
        <video
          autoPlay
          muted
          loop
          playsInline
          className="background-video"
          preload="auto"
          poster="" // Use empty poster to avoid fallback image
        >
          <source src="/vids/video.mp4" type="video/mp4" />
          Your browser does not support the video tag.
        </video>
        <div className="video-overlay"></div>
      </div>
      <div className="container" style={{position: 'relative', zIndex: 3}}>
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
            Start Reading
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
            />

            {/* Card 2: Robotic Operating System */}
            <SpotlightCard
              title="Robotic Operating System"
              description="Deep dive into ROS 2 nodes and communication."
            />

            {/* Card 3: Sim2Real Gap */}
            <SpotlightCard
              title="Sim2Real Gap"
              description="Bridge the reality gap. Techniques to deploy simulation policies onto real physical hardware."
            />
            {/* --- NEW ROW --- */}

            {/* Card 4: VLA Models */}
            <SpotlightCard
              title="VLA Models"
              description="Integration of Vision-Language-Action models."
            />

            {/* Card 5: Physical Simulation */}
            <SpotlightCard
              title="Physical Simulation"
              description="Training agents in NVIDIA Isaac Sim."
            />

            {/* Card 6: Gym & RL Training */}
            <SpotlightCard
              title="Gym & RL Training"
              description="Train robust walking policies using OmniIsaacGymEnvs and PPO/SAC algorithms."
            />

          </div>
        </div>
      </main>
    </Layout>
  );
}

export default Homepage;

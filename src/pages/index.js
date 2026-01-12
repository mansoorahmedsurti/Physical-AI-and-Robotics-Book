import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className="hero hero--primary hero--centered">
      <div className="container">
        <h1 className="hero__title">
          Physical AI & Humanoid Robotics
        </h1>
        <p className="hero__subtitle">
          From ROS 2 to Isaac Sim: The Future of Embodied Intelligence
        </p>
        {/* Video/Graphs Section */}
        <div className="hero-media">
          <div className="media-content">
            <div className="video-container has-animation">
              <video
                autoPlay
                muted
                loop
                playsInline
                className="hero-video"
                onError={(e) => {
                  e.target.classList.add('video-hidden');
                  // Show the visualization fallback
                  const vizContainer = e.target.parentNode.nextElementSibling;
                  if (vizContainer) {
                    vizContainer.style.display = 'flex';
                  }
                }}
                onLoadStart={(e) => {
                  // Hide the visualization when video starts loading
                  const vizContainer = e.target.parentNode.nextElementSibling;
                  if (vizContainer) {
                    vizContainer.style.display = 'none';
                  }
                }}
              >
                <source src="/img/robot-demo.mp4" type="video/mp4" />
                Your browser does not support the video tag.
              </video>
            </div>

            {/* Fallback: Static visualization if video fails */}
            <div className="hero-visualizations">
              <div className="visualization-grid">
                <div className="viz-item">
                  <div className="viz-icon">🤖</div>
                  <div className="viz-text">ROS 2</div>
                </div>
                <div className="viz-item">
                  <div className="viz-icon">🎮</div>
                  <div className="viz-text">Isaac Sim</div>
                </div>
                <div className="viz-item">
                  <div className="viz-icon">🧠</div>
                  <div className="viz-text">VLA Models</div>
                </div>
                <div className="viz-item">
                  <div className="viz-icon">🎯</div>
                  <div className="viz-text">RL Training</div>
                </div>
              </div>
            </div>
          </div>
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

        {/* Author Section */}
        <div className="author-section">
          <div className="author-info">
            <div className="author-avatar">
              <img src="/img/profile-placeholder.jpg" alt="Author" onError={(e) => {
                e.target.style.display = 'none';
                const parent = e.target.parentElement;
                if (parent) parent.style.display = 'none';
              }} />
            </div>
            <div className="author-details">
              <h4>By Mansoor Ahmed</h4>
              <p className="author-subtitle">AI & Robotics Engineer</p>
            </div>
          </div>
          <div className="github-stats">
            <a href="https://github.com/mansoorahmedsurti/Physical-AI-and-Robotics-Book" target="_blank" rel="noopener noreferrer">
              <span className="github-star">⭐ Star this project</span>
            </a>
          </div>
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
            <div className="card">
              <h3>Edge Deployment</h3>
              <p>
                Optimize your nodes for <strong>NVIDIA Jetson Orin</strong>. Run heavy VLA models with low latency.
              </p>
            </div>

            {/* Card 2: Robotic Operating System */}
            <div className="card">
              <h3>Robotic Operating System</h3>
              <p>Deep dive into <strong>ROS 2</strong> nodes and communication.</p>
            </div>

            {/* Card 3: Sim2Real Gap */}
            <div className="card">
              <h3>Sim2Real Gap</h3>
              <p>
                Bridge the reality gap. Techniques to deploy simulation policies onto real <strong>physical hardware</strong>.
              </p>
            </div>
            {/* --- NEW ROW --- */}

            {/* Card 4: VLA Models */}
            <div className="card">
              <h3>VLA Models</h3>
              <p>Integration of <strong>Vision-Language-Action</strong> models.</p>
            </div>

            {/* Card 5: Physical Simulation */}
            <div className="card">
              <h3>Physical Simulation</h3>
              <p>Training agents in <strong>NVIDIA Isaac Sim</strong>.</p>
            </div>

            {/* Card 6: Gym & RL Training */}
            <div className="card">
              <h3>Gym & RL Training</h3>
              <p>
                Train robust walking policies using <strong>OmniIsaacGymEnvs</strong> and PPO/SAC algorithms.
              </p>
            </div>

          </div>
        </div>
      </main>
    </Layout>
  );
}

export default Homepage;

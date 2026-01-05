import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

// This function builds the Header section
function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    // 'heroBanner' connects to your CSS .heroBanner class
    <header className="heroBanner">
      <div className="container">
        <h1 className="heroTitle">{siteConfig.title}</h1>
        <p className="heroSubtitle">{siteConfig.tagline}</p>
        <div className="buttons">
          <Link
            className="button button--primary button--lg"
            to="/docs/Intro">
            Start Learning 🚀
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

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="The Ultimate Guide to Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        {/* 'featuresSection' connects to your CSS padding */}
        <section className="featuresSection">
          {/* 'featuresGrid' connects to your CSS Grid layout */}
          <div className="featuresGrid">
            
            {/* Card 1 */}
            <div className="featureCard">
              <h3>🤖 The Nervous System</h3>
              <p>
                Master <strong>ROS 2 Jazzy</strong>. Learn how to build distributed node architectures that act as the nervous system for your humanoid robot.
              </p>
            </div>

            {/* Card 2 */}
            <div className="featureCard">
              <h3>🧠 The Digital Brain</h3>
              <p>
                Train your agents in <strong>NVIDIA Isaac Sim</strong>. Use photorealistic digital twins to safely train complex behaviors before deployment.
              </p>
            </div>

            {/* Card 3 */}
            <div className="featureCard">
              <h3>👁️ Vision-Language-Action</h3>
              <p>
                Integrate <strong>LLMs and VLA models</strong>. Teach your robot to understand natural language commands and execute physical tasks.
              </p>
            </div>

          </div>
        </section>
      </main>
    </Layout>
  );
}
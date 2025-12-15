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
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning →
          </Link>
        </div>
      </div>
    </header>
  );
}

function FeatureList() {
  const features = [
    {
      title: 'ROS 2 Nervous System',
      description: 'Learn how ROS 2 connects sensors, actuators, and AI in humanoid robots.',
    },
    {
      title: 'Digital Twin Simulation',
      description: 'Create physics-accurate simulations with Gazebo and Unity.',
    },
    {
      title: 'NVIDIA Isaac Perception',
      description: 'Integrate AI-powered perception and autonomous navigation.',
    },
    {
      title: 'Vision-Language-Action',
      description: 'Control robots with natural language using LLMs.',
    },
  ];

  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {features.map((feature, idx) => (
            <div key={idx} className={clsx('col col--3')}>
              <div className="text--center padding-horiz--md">
                <h3>{feature.title}</h3>
                <p>{feature.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics: Embodied Intelligence in the Real World">
      <HomepageHeader />
      <main>
        <FeatureList />
      </main>
    </Layout>
  );
}

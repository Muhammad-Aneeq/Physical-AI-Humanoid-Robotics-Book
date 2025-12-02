import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <Heading as="h1" className={styles.heroTitle}>
            {siteConfig.title}
          </Heading>
          <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>
          <div className={styles.heroDescription}>
            <p>
              A comprehensive course taking you from ROS 2 basics to building voice-controlled
              humanoid robots with AI perception and navigation. Perfect for software developers,
              AI engineers, and robotics enthusiasts.
            </p>
          </div>
          <div className={styles.buttons}>
            <Link
              className="button button--primary button--lg"
              to="/docs/intro">
              Start Learning 🤖
            </Link>
            <Link
              className="button button--outline button--lg"
              to="/docs/module-1"
              style={{marginLeft: '1rem'}}>
              View Modules
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="Master ROS 2, Gazebo simulation, autonomous navigation, and build voice-controlled AI-powered humanoid robots. A comprehensive course for developers and robotics enthusiasts.">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}

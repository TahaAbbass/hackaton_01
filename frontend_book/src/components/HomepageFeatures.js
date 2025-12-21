import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

// Define SVG components for the features
const UndrawDocusaurusMountain = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="175" height="175" viewBox="0 0 400 400">
    <rect width="400" height="400" fill="#2E8555"/>
    <path d="M50,350 L150,150 L250,250 L350,100 L390,350 Z" fill="#fff"/>
    <circle cx="300" cy="120" r="20" fill="#F9D342"/>
  </svg>
);

const UndrawDocusaurusReact = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="175" height="175" viewBox="0 0 400 400">
    <rect width="400" height="400" fill="#2E8555"/>
    <circle cx="200" cy="200" r="80" fill="none" stroke="#fff" stroke-width="10"/>
    <circle cx="200" cy="200" r="40" fill="#fff"/>
    <g stroke="#fff" stroke-width="5">
      <line x1="200" y1="100" x2="200" y2="140"/>
      <line x1="200" y1="260" x2="200" y2="300"/>
      <line x1="100" y1="200" x2="140" y2="200"/>
      <line x1="260" y1="200" x2="300" y2="200"/>
      <line x1="140" y1="140" x2="170" y2="170"/>
      <line x1="230" y1="230" x2="260" y2="260"/>
      <line x1="140" y1="260" x2="170" y2="230"/>
      <line x1="230" y1="170" x2="260" y2="140"/>
    </g>
  </svg>
);

const UndrawDocusaurusTree = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="175" height="175" viewBox="0 0 400 400">
    <rect width="400" height="400" fill="#2E8555"/>
    <rect x="180" y="300" width="40" height="80" fill="#8B4513"/>
    <path d="M100,300 C100,200 300,200 300,300 C250,250 150,250 100,300 Z" fill="#228B22"/>
    <circle cx="200" cy="180" r="40" fill="#228B22"/>
  </svg>
);

const FeatureList = [
  {
    title: 'Learn ROS 2',
    Svg: UndrawDocusaurusMountain,
    description: (
      <>
        Comprehensive educational content explaining ROS 2 as the middleware nervous system for humanoid robots.
      </>
    ),
  },
  {
    title: 'Communication Models',
    Svg: UndrawDocusaurusReact,
    description: (
      <>
        Understand Nodes, Topics, and Services for agent-controller communication flows in robotics.
      </>
    ),
  },
  {
    title: 'Robot Structure',
    Svg: UndrawDocusaurusTree,
    description: (
      <>
        Master URDF (Unified Robot Description Format) for describing robot structures for simulation.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
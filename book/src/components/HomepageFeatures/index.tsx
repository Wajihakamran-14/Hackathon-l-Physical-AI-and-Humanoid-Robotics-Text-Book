import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Quick Deployment & Setup',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Our system is designed for a hassle-free start. Easily install the necessary software libraries,
        connect your hardware components, and run your first AI routine in minutes. Focus on learning, not configuration.
      </>
    ),
  },
  {
    title: 'Cutting-Edge Physical AI',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
       Dive deep into the core of intelligent physical systems. Our documentation covers advanced topics like
       real-time sensor fusion, reinforcement learning algorithms, and robust motor control, allowing you to 
       concentrate on innovative AI behaviors. 
      </>
    ),
  },
  {
    title: 'Modular & Extensible Design',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Built on a modular framework, our robot architecture allows you to easily integrate custom hardware,
        develop new AI modules, and contribute to the core platform. Reuse existing code and extend functionality
        to meet your unique needs.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
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

import { FC } from "react";
import MainPage from '../pages/mainPage';
import AboutPage from '../pages/aboutPage';

export const pages: {
  [key: string]: {
    link: string;
    component: FC;
  },
} = {
  'Main': {
    link: '/',
    component: MainPage,
  },
  'About': {
    link: '/about',
    component: AboutPage,
  },
};

export const headerLinks = ['Main', 'About'];

// TODO: че-нить придумай
// https://www.typescriptlang.org/docs/handbook/2/mapped-types.html
export type headerLinksT = 'Main' | 'About';

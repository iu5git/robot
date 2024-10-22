import { FC } from "react";
import { Link } from "react-router-dom";
import { pages, headerLinks, headerLinksT } from '../../base/pages';

import logo from '../../static/logo.svg';
import './header.scss';

interface HeaderProps {
  name: headerLinksT;
}

const HeaderComponent: FC<HeaderProps> = ({name}) => (
  <nav>
    <div className="nav-wrapper theme-color">
      <a href="/" className="logo left">
        <img className="material-icons" alt='Главная' src={logo} height={56}/>
      </a>
      <ul id="nav-mobile" className="left hide-on-med-and-down">
        {headerLinks.map((sign, i) => (
          <li className={sign === name ? 'active' : ''} key={i}>
            <Link to={pages[sign].link}>{sign}</Link>
          </li>
        ))}
      </ul>
    </div>
  </nav>
);

export default HeaderComponent;

import { FC } from "react";
import { headerLinksT } from "../base/pages";
import HeaderComponent from "../components/header/headerComponent";
import WSNotifComponent from '../components/wsNotificator/wsNotificatorComponent';

interface PageProps {
  name: headerLinksT;
}

const RootPage: FC<PageProps> = (props) => (
  <>
    <HeaderComponent name={props.name}/>
    <WSNotifComponent />
    <div className="container">
      {props.children}
    </div>
  </>
);

export default RootPage;

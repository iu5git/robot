import { FC } from "react";
import ChatComponent from "../components/chat/chatComponent";
import ButtonControlPanel from '../components/buttonControlPanel/buttonControlPanelComponent';
import RootPage from "./root";

import '../index.scss'

const MainPage: FC = () => (
  <RootPage name='Main'>
    <h3 className="light-blue-text text-lighten">Панель управления</h3>
    <div className="row jc-sp-btw">
      
      <div className="plate col w-shrink">
        <ButtonControlPanel size={50} color="#29b6f6" colorHover="#03a9f4" colorPressed="#26a69a" colorPressedHover="#009688"/>
      </div>

      <div className="plate col w-60">
        <ChatComponent />
      </div>
      
    </div>
  </RootPage>
);

export default MainPage;

import { FC } from "react";
import Log from '../log/logComponent';

import { useSelector } from "react-redux";
import { selectMessages, selectOdometry } from '../../redux/reducers/wsReducer';

import '../../index.scss'

const ChatComponent: FC = () => {
  const messages = useSelector(selectMessages);
  const odometry = useSelector(selectOdometry);
  return(
    <div className="w-100">
      <form className="col s12" >
        <div className="row valign-wrapper">
          <div className="input-field col s8" style={{marginLeft: 0}}>
            <input id="msg-inpt" type="text" className="validate" />
            <label htmlFor="msg-inpt">Сообщение</label>
            <span className="helper-text" data-error="Ошибка" data-success="Отправлено">Символьные строки</span>
          </div>
          <button className="btn col s4 light-blue lighten-1" id="chat-message-submit" type="submit">
            Отправить
            {/* <i className="material-icons right">send</i> */}
          </button>
        </div>
      </form>

      <div className="divider"></div>
      <div className="section w-fill">
        <Log messages={messages}></Log>
      </div>
      
      <div className="divider"></div>
      <div className="section w-fill">
        <Log messages={odometry}></Log>
      </div>
    </div>
  
)};

export default ChatComponent;
import { FC, useState, useEffect } from "react";
import {useSelector} from "react-redux";
import {selectStatus, WSStatus} from '../../redux/reducers/wsReducer';
import WS from '../../api/webSocketRequests';

import './buttonControlPanel.scss';

interface buttonControlPanelProps {
  size: number,
  color: string,
  colorHover: string,
  colorPressed: string,
  colorPressedHover: string,
}

const cellNumsWithTriangles = [1, 3, 5, 7];

const commandPerTriangle = (idx: number) => {
  switch(idx) {
    case 1:
      WS.command.move('forward');
      break;
    case 3:
      WS.command.rotate('left');
      break;
    case 5:
      WS.command.rotate('right');
      break;
    case 7:
      WS.command.move('backward');
      break;
}};

function styleGen(color: string, hoverColor: string, rotate: number = 0): React.CSSProperties {
  return {
    "--cssHoverColor": hoverColor,
    "--cssColor": `${color}`,
    transform: `rotate(${rotate}deg)`,
  } as React.CSSProperties;
}

const ButtonControlPanelComponent: FC<buttonControlPanelProps> = (props) => {
  const [pressed, setPressed] = useState(-1);

  return(
    <div className="button_control_panel" onClick={() => {
      // if (pressed >= 0) {
      //   setPressed(-1);
      //   console.log('stop');
      // } 
    }}>
      {[...Array(3)].map((_, row_idx) => {
        return(
          <div key={row_idx} className="button_control_panel__row">
            {[...Array(3)].map((_, col_idx) => {
              const idx = row_idx * 3 + col_idx;
              const i = Math.floor(idx / 2);
              return(
                <div key={idx} className="button_control_panel__cell">
                  {
                    cellNumsWithTriangles.includes(idx) ?

                    <button
                      className="button_control_panel__triangle transparent"
                      style={styleGen(
                        pressed === i ? props.colorPressed : props.color,
                        pressed === i ? props.colorPressedHover : props.colorHover,
                        idx === 1 ? 0 : idx === 7 ? 180 : idx * 90,
                      )}
                      onClick={() => {
                        if (pressed !== i) {
                          setPressed(i);
                          commandPerTriangle(idx);
                        } else {
                          setPressed(-1);
                          WS.command.stop();
                        }
                      }}>
                    </button>
                    : idx === 4 ?
                    <button className="button_control_panel__square"
                      style={styleGen(
                        pressed === 4 ? props.colorPressed : props.color,
                        pressed === 4 ? props.colorPressedHover : props.colorHover,
                      )}
                      onClick={() => {
                        setPressed(-1);
                        WS.command.stop();
                      }}
                    >
                    </button>
                    : ''
                  }
                </div>
              )
            })}
          </div>
        );
      })}
    </div>
  );
};

export default ButtonControlPanelComponent;

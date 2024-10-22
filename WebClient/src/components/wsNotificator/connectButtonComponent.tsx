import { FC } from "react";

interface ConnectButtonProps {
  onClick: () => void,
  text: string,
}

const ConnectButtonComponent: FC<ConnectButtonProps> = (props) => {
  return(
    <button
      className="ws_notif__btn waves-effect waves-light"
      aria-label="Connect"
      onClick={props.onClick}
    >
      {props.text}
    </button>
  );
};

export default ConnectButtonComponent;
